"""Waveshare UGV Rover motor controller."""

from __future__ import annotations

import json
import time

import serial
from loguru import logger


class UGVController:
    """Sends drive commands to the Waveshare UGV Rover via serial.

    The sub-controller expects newline-terminated JSON sent at 115200 baud.
    A chassis-type command must be issued once after each power-on before
    motion commands will be accepted.

    Parameters
    ----------
    port : str
        Serial port the UGV sub-controller is connected to.
        Pi 5: ``'/dev/ttyAMA0'``, Pi 4B: ``'/dev/serial0'``.
    baud_rate : int
        Serial baud rate. Default matches the sub-controller firmware (115200).
    chassis_main : int
        Chassis type code: 1=RaspRover, 2=UGV Rover (6WD), 3=UGV Beast.
    chassis_module : int
        Attached module: 0=none, 1=arm, 2=pan-tilt.
    track_width : float
        Distance between left and right wheel centres in metres.
        Used to convert angular velocity to differential wheel speeds.
    """

    def __init__(
        self,
        port: str = "/dev/ttyAMA0",
        baud_rate: int = 115200,
        chassis_main: int = 2,
        chassis_module: int = 0,
        track_width: float = 0.3,
    ) -> None:
        self._port = port
        self._baud_rate = baud_rate
        self._chassis_main = chassis_main
        self._chassis_module = chassis_module
        self._track_width = track_width
        self._serial: serial.Serial | None = None
        logger.debug(
            f"UGVController initialised (port={port}, baud={baud_rate}, "
            f"chassis_main={chassis_main}, track_width={track_width} m)."
        )

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _send(self, command: dict[str, object]) -> None:
        """JSON-encode *command* and write it to the serial port."""
        if self._serial is None or not self._serial.is_open:
            raise RuntimeError("Serial port is not open. Call connect() first.")
        payload = json.dumps(command, separators=(",", ":")) + "\n"
        self._serial.write(payload.encode("utf-8"))
        logger.debug(f"Sent: {payload.strip()}")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Open the serial connection and configure the chassis type.

        Must be called once after each power-on before issuing motion commands.
        """
        logger.info(f"Connecting to UGV on {self._port} at {self._baud_rate} baud...")
        self._serial = serial.Serial(self._port, self._baud_rate, timeout=1)
        time.sleep(0.1)  # Allow ESP32 to settle after port open
        self._send(
            {"T": 900, "main": self._chassis_main, "module": self._chassis_module}
        )
        logger.info("UGV connected and chassis type set.")

    def disconnect(self) -> None:
        """Stop the UGV and close the serial connection."""
        logger.info("Disconnecting UGV...")
        if self._serial is not None and self._serial.is_open:
            self.stop()
            self._serial.close()
            self._serial = None
        logger.info("UGV disconnected.")

    def move(self, linear: float, angular: float) -> None:
        """Send a velocity command to the UGV.

        Uses differential steering: the left/right wheel speeds are derived
        from the desired linear and angular velocities.

        Parameters
        ----------
        linear : float
            Forward/backward speed in m/s. Positive = forward.
        angular : float
            Rotational speed in rad/s. Positive = left (counter-clockwise).
        """
        half_track = self._track_width / 2.0
        left = linear - angular * half_track
        right = linear + angular * half_track
        self._send({"T": 1, "L": round(left, 4), "R": round(right, 4)})

    def stop(self) -> None:
        """Send a zero-velocity command to halt the UGV."""
        self._send({"T": 1, "L": 0, "R": 0})
        logger.info("UGV stopped.")
