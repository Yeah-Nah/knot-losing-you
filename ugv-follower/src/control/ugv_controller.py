"""Waveshare UGV Rover motor controller."""

from __future__ import annotations

from loguru import logger


class UGVController:
    """Sends drive commands to the Waveshare UGV Rover via serial.

    Parameters
    ----------
    port : str
        Serial port the UGV controller is connected to (e.g. ``'/dev/ttyAMA0'``).
    """

    def __init__(self, port: str = "/dev/ttyAMA0") -> None:
        self._port = port
        logger.debug("UGVController initialised (stub).")

    def connect(self) -> None:
        """Open the serial connection to the UGV controller."""
        raise NotImplementedError

    def disconnect(self) -> None:
        """Close the serial connection."""
        raise NotImplementedError

    def move(self, linear: float, angular: float) -> None:
        """Send a velocity command to the UGV.

        Parameters
        ----------
        linear : float
            Forward/backward speed in m/s. Positive = forward.
        angular : float
            Rotational speed in rad/s. Positive = left (counter-clockwise).
        """
        raise NotImplementedError

    def stop(self) -> None:
        """Send a zero-velocity command to halt the UGV."""
        raise NotImplementedError
