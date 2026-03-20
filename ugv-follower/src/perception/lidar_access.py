"""LiDAR sensor access layer."""

from __future__ import annotations

from loguru import logger


class LidarAccess:
    """Manages the LiDAR sensor connection and scan retrieval.

    Parameters
    ----------
    port : str
        Serial port the LiDAR is connected to (e.g. ``'/dev/ttyUSB0'``).
    baud_rate : int
        Serial baud rate for the LiDAR connection.
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baud_rate: int = 115200) -> None:
        self._port = port
        self._baud_rate = baud_rate
        logger.debug("LidarAccess initialised (stub).")

    def start(self) -> None:
        """Open the LiDAR connection and begin scanning."""
        raise NotImplementedError

    def stop(self) -> None:
        """Stop scanning and close the LiDAR connection."""
        raise NotImplementedError
