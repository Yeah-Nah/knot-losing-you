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
from typing import TYPE_CHECKING

import serial
from loguru import logger

if TYPE_CHECKING:
    pass

# ------------------------------------------------------------------
# Protocol constants
# ------------------------------------------------------------------

_PACKET_SIZE = 47
_HEADER = 0x54
_VER_LEN = 0x2C
_POINTS_PER_PACK = 12
_POINT_STRUCT = struct.Struct("<HB")  # uint16 distance + uint8 intensity
_PKG_STRUCT = struct.Struct("<BBHHH")  # header ver_len speed start_angle (+ points) end_angle timestamp

# CRC-8/MAXIM lookup table (polynomial 0x31, init 0x00)
_CRC_TABLE: tuple[int, ...] = (
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae,
    0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1,
    0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18,
    0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39,
    0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f,
    0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2,
    0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b,
    0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64,
    0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec,
    0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3,
    0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a,
    0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8,
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

    def _read_packet(self) -> list[dict] | None:
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
        speed_raw = struct.unpack_from("<H", packet, 2)[0]   # degrees/sec
        start_angle = struct.unpack_from("<H", packet, 4)[0] / 100.0
        end_angle = struct.unpack_from("<H", packet, 42)[0] / 100.0

        # Unwrap end_angle if it wraps past 360°
        if end_angle < start_angle:
            end_angle += 360.0

        angle_step = (end_angle - start_angle) / (_POINTS_PER_PACK - 1)

        points = []
        offset = 6  # points start at byte 6
        for i in range(_POINTS_PER_PACK):
            distance, intensity = _POINT_STRUCT.unpack_from(packet, offset)
            angle = (start_angle + i * angle_step) % 360.0
            points.append({"angle": round(angle, 2), "distance": distance, "intensity": intensity})
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

    def get_scan(self) -> list[dict] | None:
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
