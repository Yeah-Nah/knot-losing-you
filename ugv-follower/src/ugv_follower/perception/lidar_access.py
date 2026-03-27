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

# CRC-8/MAXIM lookup table (polynomial 0x31, init 0x00)
_CRC_TABLE: tuple[int, ...] = (
    0x00,
    0x4D,
    0x9A,
    0xD7,
    0x79,
    0x34,
    0xE3,
    0xAE,
    0xF2,
    0xBF,
    0x68,
    0x25,
    0x8B,
    0xC6,
    0x11,
    0x5C,
    0xA9,
    0xE4,
    0x33,
    0x7E,
    0xD0,
    0x9D,
    0x4A,
    0x07,
    0x5B,
    0x16,
    0xC1,
    0x8C,
    0x22,
    0x6F,
    0xB8,
    0xF5,
    0x1F,
    0x52,
    0x85,
    0xC8,
    0x66,
    0x2B,
    0xFC,
    0xB1,
    0xED,
    0xA0,
    0x77,
    0x3A,
    0x94,
    0xD9,
    0x0E,
    0x43,
    0xB6,
    0xFB,
    0x2C,
    0x61,
    0xCF,
    0x82,
    0x55,
    0x18,
    0x44,
    0x09,
    0xDE,
    0x93,
    0x3D,
    0x70,
    0xA7,
    0xEA,
    0x3E,
    0x73,
    0xA4,
    0xE9,
    0x47,
    0x0A,
    0xDD,
    0x90,
    0xCC,
    0x81,
    0x56,
    0x1B,
    0xB5,
    0xF8,
    0x2F,
    0x62,
    0x97,
    0xDA,
    0x0D,
    0x40,
    0xEE,
    0xA3,
    0x74,
    0x39,
    0x65,
    0x28,
    0xFF,
    0xB2,
    0x1C,
    0x51,
    0x86,
    0xCB,
    0x21,
    0x6C,
    0xBB,
    0xF6,
    0x58,
    0x15,
    0xC2,
    0x8F,
    0xD3,
    0x9E,
    0x49,
    0x04,
    0xAA,
    0xE7,
    0x30,
    0x7D,
    0x88,
    0xC5,
    0x12,
    0x5F,
    0xF1,
    0xBC,
    0x6B,
    0x26,
    0x7A,
    0x37,
    0xE0,
    0xAD,
    0x03,
    0x4E,
    0x99,
    0xD4,
    0x7C,
    0x31,
    0xE6,
    0xAB,
    0x05,
    0x48,
    0x9F,
    0xD2,
    0x8E,
    0xC3,
    0x14,
    0x59,
    0xF7,
    0xBA,
    0x6D,
    0x20,
    0xD5,
    0x98,
    0x4F,
    0x02,
    0xAC,
    0xE1,
    0x36,
    0x7B,
    0x27,
    0x6A,
    0xBD,
    0xF0,
    0x5E,
    0x13,
    0xC4,
    0x89,
    0x63,
    0x2E,
    0xF9,
    0xB4,
    0x1A,
    0x57,
    0x80,
    0xCD,
    0x91,
    0xDC,
    0x0B,
    0x46,
    0xE8,
    0xA5,
    0x72,
    0x3F,
    0xCA,
    0x87,
    0x50,
    0x1D,
    0xB3,
    0xFE,
    0x29,
    0x64,
    0x38,
    0x75,
    0xA2,
    0xEF,
    0x41,
    0x0C,
    0xDB,
    0x96,
    0x42,
    0x0F,
    0xD8,
    0x95,
    0x3B,
    0x76,
    0xA1,
    0xEC,
    0xB0,
    0xFD,
    0x2A,
    0x67,
    0xC9,
    0x84,
    0x53,
    0x1E,
    0xEB,
    0xA6,
    0x71,
    0x3C,
    0x92,
    0xDF,
    0x08,
    0x45,
    0x19,
    0x54,
    0x83,
    0xCE,
    0x60,
    0x2D,
    0xFA,
    0xB7,
    0x5D,
    0x10,
    0xC7,
    0x8A,
    0x24,
    0x69,
    0xBE,
    0xF3,
    0xAF,
    0xE2,
    0x35,
    0x78,
    0xD6,
    0x9B,
    0x4C,
    0x01,
    0xF4,
    0xB9,
    0x6E,
    0x23,
    0x8D,
    0xC0,
    0x17,
    0x5A,
    0x06,
    0x4B,
    0x9C,
    0xD1,
    0x7F,
    0x32,
    0xE5,
    0xA8,
)


def _crc8(data: bytes) -> int:
    """Compute CRC-8/MAXIM over *data*."""
    crc = 0
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

        # Sync to header: scan byte-by-byte for 0x54 followed by 0x2C
        while True:
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

        # Verify CRC (over bytes 0-45, result compared to byte 46)
        if _crc8(packet[:46]) != packet[46]:
            logger.warning("LiDAR packet CRC mismatch — discarding.")
            return None

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
                {"angle": round(angle, 2), "distance": distance, "intensity": intensity}
            )
            offset += 3

        return points

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
        self._serial = serial.Serial(self._port, self._baud_rate, timeout=1)
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
