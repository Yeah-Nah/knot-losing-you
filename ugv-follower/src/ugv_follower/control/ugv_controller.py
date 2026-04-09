"""Waveshare UGV Rover motor controller."""

from __future__ import annotations

import json
import time

import serial
from loguru import logger

from .command_shaper import CommandShaper


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
    shaping_enabled : bool
        When ``True``, wheel commands are shaped by a background
        :class:`~ugv_follower.control.command_shaper.CommandShaper` thread
        before reaching hardware.  Defaults to ``False`` so calibration
        scripts that manage their own timing are unaffected.
    update_rate_hz : float
        Shaper loop frequency in Hz.  Ignored when *shaping_enabled* is
        ``False``.
    ramp_rate_per_s : float
        Maximum wheel speed change per second (m/s per s).  Ignored when
        *shaping_enabled* is ``False``.
    reversal_dwell_s : float
        Seconds to hold a wheel at zero after a sign crossing before
        accelerating in the opposite direction.  Ignored when
        *shaping_enabled* is ``False``.
    zero_crossing_epsilon : float
        Wheel speeds with ``|v| <= epsilon`` are treated as zero for
        reversal detection.  Ignored when *shaping_enabled* is ``False``.
    """

    def __init__(
        self,
        port: str = "/dev/ttyAMA0",
        baud_rate: int = 115200,
        chassis_main: int = 2,
        chassis_module: int = 0,
        track_width: float = 0.3,
        shaping_enabled: bool = False,
        update_rate_hz: float = 50.0,
        ramp_rate_per_s: float = 2.0,
        reversal_dwell_s: float = 0.05,
        zero_crossing_epsilon: float = 0.01,
    ) -> None:
        self._port = port
        self._baud_rate = baud_rate
        self._chassis_main = chassis_main
        self._chassis_module = chassis_module
        self._track_width = track_width
        self._serial: serial.Serial | None = None
        self._shaper: CommandShaper | None = (
            CommandShaper(
                send_fn=self._send_wheel_speeds,
                update_rate_hz=update_rate_hz,
                ramp_rate_per_s=ramp_rate_per_s,
                reversal_dwell_s=reversal_dwell_s,
                zero_crossing_epsilon=zero_crossing_epsilon,
            )
            if shaping_enabled
            else None
        )
        logger.debug(
            f"UGVController initialised (port={port}, baud={baud_rate}, "
            f"chassis_main={chassis_main}, track_width={track_width} m, "
            f"shaping={'on' if shaping_enabled else 'off'})."
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

    def _send_wheel_speeds(self, left: float, right: float) -> None:
        """Send pre-computed wheel speeds directly to hardware.

        Parameters
        ----------
        left : float
            Left wheel speed in m/s.
        right : float
            Right wheel speed in m/s.
        """
        self._send({"T": 1, "L": round(left, 4), "R": round(right, 4)})

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Open the serial connection and configure the chassis type.

        Must be called once after each power-on before issuing motion commands.
        Starts the command-shaping thread if shaping is enabled.
        """
        logger.info(f"Connecting to UGV on {self._port} at {self._baud_rate} baud...")
        self._serial = serial.Serial(self._port, self._baud_rate, timeout=1)
        time.sleep(0.1)  # Allow ESP32 to settle after port open
        self._send(
            {"T": 900, "main": self._chassis_main, "module": self._chassis_module}
        )
        if self._shaper is not None:
            self._shaper.start()
        logger.info("UGV connected and chassis type set.")

    def disconnect(self) -> None:
        """Stop the UGV, shut down the shaper thread, and close the serial connection."""
        logger.info("Disconnecting UGV...")
        if self._serial is not None and self._serial.is_open:
            self.stop()
            if self._shaper is not None:
                self._shaper.stop()
            self._serial.close()
            self._serial = None
        logger.info("UGV disconnected.")

    def move(self, linear: float, angular: float) -> None:
        """Send a velocity command to the UGV.

        Uses differential steering: the left/right wheel speeds are derived
        from the desired linear and angular velocities.

        When command shaping is enabled, this method updates the shaper's
        target and returns immediately; the background thread applies rate
        limiting and reversal protection before sending to hardware.  When
        shaping is disabled, the command is written to the serial port
        directly.

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
        if self._shaper is not None:
            self._shaper.set_target(left, right)
        else:
            self._send_wheel_speeds(left, right)

    def stop(self) -> None:
        """Halt the UGV immediately, bypassing command shaping.

        When shaping is enabled, the shaper state (current speeds, targets,
        dwell counters) is also reset so the rover does not resume motion
        after the stop.  A zero-velocity command is always written to the
        serial port directly.
        """
        if self._shaper is not None:
            self._shaper.immediate_stop()
        else:
            self._send({"T": 1, "L": 0, "R": 0})
        logger.info("UGV stopped.")

    def set_pan_tilt(self, x_deg: float, y_deg: float) -> None:
        """Command the pan-tilt module to the given angles in degrees.

        Parameters
        ----------
        x_deg : float
            Pan angle in degrees. 0 = centre.
        y_deg : float
            Tilt angle in degrees. 0 = centre.
        """
        self._send({"T": 133, "X": x_deg, "Y": y_deg, "SPD": 0, "ACC": 0})
        logger.info(f"Pan-tilt set to ({x_deg}°, {y_deg}°).")
