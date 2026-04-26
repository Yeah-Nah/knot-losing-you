"""LDRobot D500 LiDAR sensor access layer.

Parses the LD06/LD19/D500 binary streaming protocol directly via pyserial.
The device streams packets continuously with no command needed to start.

Packet layout (47 bytes, little-endian)
---------------------------------------
  0      header       0x54
  1      ver_len      0x2C
  2-3    speed        rotation speed (degrees/sec), uint16
  4-5    start_angle  start angle × 100 (degrees), uint16
  6-41   points[12]   each: uint16 distance (mm) + uint8 intensity
  42-43  end_angle    end angle × 100 (degrees), uint16
  44-45  timestamp    milliseconds, uint16
  46     crc8         CRC-8/MAXIM over bytes 0-45
"""

from __future__ import annotations

import struct
import time
from typing import TypedDict

import serial
from loguru import logger


class LidarPoint(TypedDict):
    """A single distance measurement from the LDRobot D500."""

    angle: float
    distance: int
    intensity: int


# ------------------------------------------------------------------
# Protocol constants
# ------------------------------------------------------------------

_PACKET_SIZE = 47
_HEADER = 0x54
_VER_LEN = 0x2C
_POINTS_PER_PACK = 12
_POINT_STRUCT = struct.Struct("<HB")  # uint16 distance + uint8 intensity
_PKG_STRUCT = struct.Struct(
    "<BBHHH"
)  # header ver_len speed start_angle (+ points) end_angle timestamp
_SERIAL_TIMEOUT_S = 0.1

# CRC-8/MAXIM constants (polynomial 0x31, init 0x00)
_CRC8_POLYNOMIAL = 0x31
_CRC8_INIT = 0x00


def _build_crc8_table(polynomial: int) -> tuple[int, ...]:
    """Build a CRC-8/MAXIM lookup table for the given polynomial.

    Parameters
    ----------
    polynomial : int
        CRC-8 polynomial in normal form (for MAXIM, ``0x31``).

    Returns
    -------
    tuple[int, ...]
        256-entry lookup table used for byte-wise CRC updates.
    """
    table: list[int] = []
    for value in range(256):
        crc = value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ polynomial) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        table.append(crc)
    return tuple(table)


_CRC_TABLE: tuple[int, ...] = _build_crc8_table(_CRC8_POLYNOMIAL)


def _crc8(data: bytes) -> int:
    """Compute CRC-8/MAXIM over *data*."""
    crc = _CRC8_INIT
    for byte in data:
        crc = _CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc


class LidarAccess:
    """Manages the LDRobot D500 LiDAR connection and packet retrieval.

    The D500 streams 47-byte packets continuously at 230400 baud with no
    start command required.  Each packet contains 12 distance points covering
    a ~30° arc.  Assembling a full 360° scan is left to the pipeline layer.

    Parameters
    ----------
    port : str
        Serial port the LiDAR is connected to (e.g. ``'/dev/ttyUSB0'``).
    baud_rate : int
        Serial baud rate. Default matches the D500 firmware (230400).
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baud_rate: int = 230400) -> None:
        self._port = port
        self._baud_rate = baud_rate
        self._serial: serial.Serial | None = None
        logger.debug(f"LidarAccess initialised (port={port}, baud={baud_rate}).")

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _read_packet(self) -> list[LidarPoint] | None:
        """Block until one valid packet is received and return its 12 points.

        Scans the byte stream for the two-byte sync header (0x54 0x2C),
        reads the remaining 45 bytes, verifies the CRC, then parses and
        returns the 12 measurement points.

        Returns
        -------
        list[dict] | None
            List of 12 dicts with keys ``angle`` (float, degrees),
            ``distance`` (int, mm), and ``intensity`` (int, 0-255),
            or ``None`` if a CRC error occurs or the port times out.
        """
        if self._serial is None or not self._serial.is_open:
            raise RuntimeError("Serial port is not open. Call start() first.")

        # Keep scanning until a valid packet is decoded or we hit the serial timeout.
        timeout_s = float(self._serial.timeout or 1.0)
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            # Sync to header: scan byte-by-byte for 0x54 followed by 0x2C
            while True:
                if time.monotonic() >= deadline:
                    return None
                byte = self._serial.read(1)
                if not byte:
                    return None  # timeout
                if byte[0] != _HEADER:
                    continue
                next_byte = self._serial.read(1)
                if not next_byte:
                    return None
                if next_byte[0] == _VER_LEN:
                    break  # found sync

            # Read remaining 45 bytes (we already consumed the first 2)
            rest = self._serial.read(_PACKET_SIZE - 2)
            if len(rest) < _PACKET_SIZE - 2:
                return None  # timeout mid-packet

            packet = bytes([_HEADER, _VER_LEN]) + rest

            # Verify CRC (over bytes 0-45, result compared to byte 46).
            # A mismatch can occur while syncing mid-stream; keep scanning.
            if _crc8(packet[:46]) != packet[46]:
                logger.debug("LiDAR packet CRC mismatch — discarding.")
                continue

            # Parse header fields
            start_angle = struct.unpack_from("<H", packet, 4)[0] / 100.0
            end_angle = struct.unpack_from("<H", packet, 42)[0] / 100.0

            # Unwrap end_angle if it wraps past 360°
            if end_angle < start_angle:
                end_angle += 360.0

            angle_step = (end_angle - start_angle) / (_POINTS_PER_PACK - 1)

            points: list[LidarPoint] = []
            offset = 6  # points start at byte 6
            for i in range(_POINTS_PER_PACK):
                distance, intensity = _POINT_STRUCT.unpack_from(packet, offset)
                angle = (start_angle + i * angle_step) % 360.0
                points.append(
                    {
                        "angle": round(angle, 2),
                        "distance": distance,
                        "intensity": intensity,
                    }
                )
                offset += 3

            return points

        return None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Open the LiDAR serial connection and begin scanning.

        Raises
        ------
        serial.SerialException
            If the port cannot be opened.
        """
        logger.info(f"Starting LiDAR on {self._port} at {self._baud_rate} baud...")
        self._serial = serial.Serial(
            self._port,
            self._baud_rate,
            timeout=_SERIAL_TIMEOUT_S,
        )
        self._serial.reset_input_buffer()
        logger.info("LiDAR started.")

    def get_scan(self) -> list[LidarPoint] | None:
        """Return the next 12-point packet from the LiDAR stream.

        Blocks until one valid packet arrives (up to the serial timeout).

        Returns
        -------
        list[dict] | None
            List of 12 dicts: ``{angle: float, distance: int, intensity: int}``.
            ``distance`` is in millimetres; 0 means out-of-range.
            Returns ``None`` on timeout or CRC error.
        """
        return self._read_packet()

    def stop(self) -> None:
        """Stop scanning and close the LiDAR serial connection."""
        logger.info("Stopping LiDAR...")
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        self._serial = None
        logger.info("LiDAR stopped.")
