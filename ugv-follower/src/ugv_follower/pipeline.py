"""Pipeline orchestration for ugv-follower.

This module coordinates perception, inference, and control
for the autonomous UGV follower.
"""

from __future__ import annotations

from enum import StrEnum
from typing import TYPE_CHECKING

from loguru import logger

from .control.motion_command import (
    MotionCommand,
    MotionCommandSource,
    apply_motion_command,
)
from .control.pan_controller import PanController
from .control.ugv_controller import UGVController
from .inference.object_detection import ObjectDetection, select_target_centroid
from .perception.camera_access import CameraAccess
from .perception.lidar_access import LidarAccess
from .perception.lidar_geometry import (
    filter_forward_arc,
    lidar_point_to_body_frame,
    nearest_forward_point,
)
from .utils.fisheye_utils import load_fisheye_intrinsics

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

    from .settings import Settings


class PipelineMode(StrEnum):
    """Minimal control modes for Phase 3A-lite."""

    AUTONOMOUS = "autonomous"
    MANUAL = "manual"


class Pipeline:
    """Orchestrates perception, inference, and UGV control.

    Parameters
    ----------
    settings : Settings
        Fully resolved and validated settings instance.
    """

    def __init__(self, settings: Settings) -> None:
        self._settings = settings
        self._mode = PipelineMode.AUTONOMOUS
        self._mode_transition_stop_pending = False
        self._estop_active = False
        self._loop_period_s = 0.1
        self._thin_run_tick = 0
        self._thin_run_complete = False
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
        K, D = load_fisheye_intrinsics(settings.sensor_config)
        self._pan_controller = PanController(
            K=K,
            D=D,
            cmd_min_deg=settings.pan_cmd_min_deg,
            cmd_max_deg=settings.pan_cmd_max_deg,
            dead_band_pos_deg=settings.pan_dead_band_pos_deg,
            dead_band_neg_deg=settings.pan_dead_band_neg_deg,
            tilt_deg=settings.pan_tilt_setpoint_deg,
        )
        self._detector: ObjectDetection | None = (
            ObjectDetection(settings.model_path, settings.model_config)
            if settings.inference_enabled
            else None
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

        # Home pan-tilt before perception starts so camera begins centered.
        self._ugv.set_pan_tilt(0.0, self._settings.pan_tilt_setpoint_deg)
        logger.info("Pan-tilt homed to startup setpoint.")

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
        logger.info("Thin 3A-lite run started. Press Ctrl+C to exit.")

        # Thin 3A-lite run: exercise the safe command path on real hardware.
        while True:
            self._advance_thin_run_scenario()
            self._update_lidar_state()
            cmd = self._decide_command()
            cmd = self._apply_mode_transition_stop(cmd)
            cmd = self._apply_estop_override(cmd)
            self._log_motion_command(cmd)
            self._apply_motion_command(cmd)
            frame = self._camera.get_frame()
            bbox_u, bbox_v = self._detect_target(frame)
            self._update_pan_state(bbox_u, bbox_v)
            if self._thin_run_complete:
                logger.info("Thin 3A-lite run finished; exiting main loop.")
                break
            time.sleep(self._loop_period_s)

    def _update_lidar_state(self) -> None:
        """Read one LiDAR packet, convert to body frame, and log the nearest forward distance."""
        scan = self._lidar.get_scan()
        if scan is None:
            return
        body = [
            lidar_point_to_body_frame(
                p,
                self._settings.lidar_mounting_offset_deg,
                self._settings.lidar_forward_displacement_m,
            )
            for p in scan
        ]
        forward = filter_forward_arc(body)
        nearest = nearest_forward_point(forward)
        if nearest is not None:
            logger.debug(
                "lidar nearest_forward dist={:.3f} m  bearing={:.1f} deg",
                nearest["distance_m"],
                nearest["bearing_deg"],
            )

    def _decide_command(self) -> MotionCommand:
        """Sole decision point for chassis motion commands.

        All Phase 3 logic (detection, LiDAR ranging, mode selection) will be
        added here. For now returns a zero-velocity hold until sensors and
        control loops are wired in.
        """
        if self._mode == PipelineMode.MANUAL:
            return MotionCommand.zero(
                source=MotionCommandSource.MANUAL,
                reason="hold_manual_mode",
            )
        return MotionCommand.zero(
            source=MotionCommandSource.AUTONOMOUS,
            reason="hold_autonomous_mode",
        )

    def set_mode(self, new_mode: PipelineMode) -> None:
        """Switch control mode and force a zero command on transition."""
        if new_mode == self._mode:
            return
        old_mode = self._mode
        self._mode = new_mode
        self._mode_transition_stop_pending = True
        logger.info(f"Mode changed: {old_mode.value} -> {new_mode.value}")

    def _apply_mode_transition_stop(self, command: MotionCommand) -> MotionCommand:
        """Emit a mandatory zero command once after every mode transition."""
        if self._mode_transition_stop_pending:
            self._mode_transition_stop_pending = False
            if self._mode == PipelineMode.MANUAL:
                return MotionCommand.zero(
                    source=MotionCommandSource.MANUAL,
                    reason="mode_transition_stop",
                )
            return MotionCommand.zero(
                source=MotionCommandSource.AUTONOMOUS,
                reason="mode_transition_stop",
            )
        return command

    def _apply_motion_command(self, command: MotionCommand) -> None:
        """Apply one normalized motion command to the controller."""
        apply_motion_command(self._ugv, command)

    def _detect_target(
        self,
        frame: NDArray[np.uint8] | None,
    ) -> tuple[float | None, float | None]:
        """Return the best detection centroid ``(u, v)`` or ``(None, None)``.

        Parameters
        ----------
        frame : NDArray[np.uint8] | None
            Current camera frame, or ``None`` if no frame is available.

        Returns
        -------
        tuple[float | None, float | None]
            Pixel centroid of the highest-confidence detection, or
            ``(None, None)`` when inference is disabled or no detection found.
        """
        if self._detector is None or frame is None:
            return None, None
        results = self._detector.run(frame)
        return select_target_centroid(results)

    def _update_pan_state(
        self,
        bbox_centre_u: float | None,
        bbox_centre_v: float | None,
    ) -> None:
        """Update the pan servo from a detection centroid, or hold if no detection.

        Parameters
        ----------
        bbox_centre_u : float | None
            Horizontal pixel coordinate of the bounding-box centroid, or
            ``None`` when there is no valid detection.
        bbox_centre_v : float | None
            Vertical pixel coordinate of the bounding-box centroid, or
            ``None`` when there is no valid detection.
        """
        pan_cmd = self._pan_controller.update(bbox_centre_u, bbox_centre_v)
        if pan_cmd is not None:
            self._ugv.set_pan_tilt(pan_cmd, self._settings.pan_tilt_setpoint_deg)

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
            return MotionCommand.zero(
                source=MotionCommandSource.ESTOP,
                reason="estop_override",
            )
        return command

    def _advance_thin_run_scenario(self) -> None:
        """Exercise one safe mode/estop sequence during the thin real run."""
        if self._thin_run_complete:
            return

        self._thin_run_tick += 1

        if self._thin_run_tick == 5:
            self.set_mode(PipelineMode.MANUAL)
        elif self._thin_run_tick == 10:
            self.request_estop()
        elif self._thin_run_tick == 15:
            self.clear_estop()
        elif self._thin_run_tick == 16:
            self.set_mode(PipelineMode.AUTONOMOUS)
        elif self._thin_run_tick == 20:
            self._thin_run_complete = True
            logger.info("Thin 3A-lite scenario complete.")

    def _log_motion_command(self, command: MotionCommand) -> None:
        """Log the emitted command so thin-run edge cases are visible."""
        logger.info(
            "cmd mode={} estop={} source={} linear_m_s={:.3f} angular_rad_s={:.3f} reason={}",
            self._mode.value,
            self._estop_active,
            command.source.value,
            command.linear_m_s,
            command.angular_rad_s,
            command.reason or "unspecified",
        )

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
