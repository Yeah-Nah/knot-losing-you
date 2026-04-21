"""Pipeline orchestration for ugv-follower.

This module coordinates perception, inference, and control
for the autonomous UGV follower.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from loguru import logger

from .control.motion_command import (
    MotionCommand,
    MotionCommandSource,
    apply_motion_command,
)
from .control.ugv_controller import UGVController
from .perception.camera_access import CameraAccess
from .perception.lidar_access import LidarAccess

if TYPE_CHECKING:
    from .settings import Settings


class Pipeline:
    """Orchestrates perception, inference, and UGV control.

    Parameters
    ----------
    settings : Settings
        Fully resolved and validated settings instance.
    """

    def __init__(self, settings: Settings) -> None:
        self._settings = settings
        self._estop_active = False
        self._camera = CameraAccess(
            fps=settings.camera_fps,
            resolution=settings.camera_resolution,
        )
        self._lidar = LidarAccess(
            port=settings.lidar_port,
            baud_rate=settings.lidar_baud_rate,
        )
        self._ugv = UGVController(
            port=settings.ugv_port,
            baud_rate=settings.ugv_baud_rate,
            chassis_main=settings.ugv_chassis_main,
            chassis_module=settings.ugv_chassis_module,
            track_width=settings.ugv_control_track_width_m,
            shaping_enabled=settings.ugv_shaping_enabled,
            update_rate_hz=settings.ugv_shaping_update_rate_hz,
            ramp_rate_per_s=settings.ugv_shaping_ramp_rate_per_s,
            reversal_dwell_s=settings.ugv_shaping_reversal_dwell_s,
            zero_crossing_epsilon=settings.ugv_shaping_zero_crossing_epsilon,
        )
        logger.info("Pipeline initialised.")

    def run(self) -> None:
        """Start the pipeline and enter the main processing loop."""
        logger.info("Starting pipeline...")
        try:
            self._main_loop()
        except KeyboardInterrupt:
            logger.info("Pipeline interrupted by user.")
        finally:
            self._shutdown()

    def _main_loop(self) -> None:
        """Smoke test: verify all three hardware components are operational."""

        # -- UGV controller --
        self._ugv.connect()
        logger.success("controller connected ✓")

        # -- Camera: wait for first frame --
        self._camera.start()
        frame = None
        timeout = 5.0  # seconds
        import time  # noqa: PLC0415

        start = time.monotonic()
        while frame is None:
            frame = self._camera.get_frame()
            if frame is None:
                time.sleep(0.01)  # 10ms backoff
            if (time.monotonic() - start) >= timeout:
                raise TimeoutError(f"Camera failed to produce frame within {timeout}s")
        logger.success(f"frame received ✓  shape={frame.shape}  dtype={frame.dtype}")

        # -- LiDAR: wait for first packet --
        self._lidar.start()
        scan = None
        timeout = 5.0  # seconds
        start = time.monotonic()
        while scan is None:
            scan = self._lidar.get_scan()
            if scan is None:
                time.sleep(0.01)  # 10ms backoff to avoid busy-spin
            if (time.monotonic() - start) >= timeout:
                raise TimeoutError(
                    "LiDAR failed to produce scan within "
                    f"{timeout}s (port={self._settings.lidar_port}, "
                    "check cable/power/port)."
                )
        valid_distances = [p["distance"] for p in scan if p["distance"] > 0]
        min_d = min(valid_distances) if valid_distances else 0
        logger.success(
            f"scan received ✓  points={len(scan)}  "
            f"valid={len(valid_distances)}  min={min_d} mm"
        )

        logger.success("All sensors operational ✓")
        logger.info("Press Ctrl+C to exit.")

        # Keep alive so the operator can observe the log output.
        while True:
            # Phase 3A-lite Step 2: one decision point feeds one apply call.
            cmd = self._decide_command()
            cmd = self._apply_estop_override(cmd)
            self._apply_motion_command(cmd)
            time.sleep(1)

    def _decide_command(self) -> MotionCommand:
        """Sole decision point for chassis motion commands.

        All Phase 3 logic (detection, LiDAR ranging, mode selection) will be
        added here. For now returns a zero-velocity hold until sensors and
        control loops are wired in.
        """
        return MotionCommand.zero(source=MotionCommandSource.AUTONOMOUS)

    def _apply_motion_command(self, command: MotionCommand) -> None:
        """Apply one normalized motion command to the controller."""
        apply_motion_command(self._ugv, command)

    def request_estop(self) -> None:
        """Latch emergency-stop override until explicitly cleared."""
        self._estop_active = True
        logger.warning("Emergency-stop requested; motion commands now overridden.")

    def clear_estop(self) -> None:
        """Clear emergency-stop override latch."""
        self._estop_active = False
        logger.info("Emergency-stop cleared.")

    def _apply_estop_override(self, command: MotionCommand) -> MotionCommand:
        """Override all motion commands when emergency-stop is active."""
        if self._estop_active:
            return MotionCommand.zero(source=MotionCommandSource.ESTOP)
        return command

    def _shutdown(self) -> None:
        """Release all hardware resources on exit."""
        logger.info("Shutting down pipeline...")
        try:
            self._camera.stop()
        except Exception as exc:
            logger.warning(f"Camera stop error: {exc}")
        try:
            self._lidar.stop()
        except Exception as exc:
            logger.warning(f"LiDAR stop error: {exc}")
        try:
            self._ugv.disconnect()
        except Exception as exc:
            logger.warning(f"UGV disconnect error: {exc}")
        logger.info("Pipeline shut down.")
