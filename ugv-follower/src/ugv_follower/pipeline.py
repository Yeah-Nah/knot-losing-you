"""Pipeline orchestration for ugv-follower.

This module coordinates perception, inference, and control
for the autonomous UGV follower.
"""

from __future__ import annotations

from enum import StrEnum
from typing import TYPE_CHECKING, Any

from loguru import logger

from .control.motion_command import (
    MotionCommand,
    MotionCommandSource,
    apply_motion_command,
)
from .control.pan_controller import PanController
from .control.ugv_controller import UGVController
from .inference.object_detection import ObjectDetection, select_target_centroid
from .perception.waveshare_camera import WaveshareCamera
from .perception.lidar_access import LidarAccess
from .perception.lidar_geometry import (
    filter_forward_arc,
    lidar_point_to_body_frame,
    nearest_forward_point,
)
from .utils.fisheye_utils import load_fisheye_intrinsics
from .utils.mjpeg_server import MjpegServer

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
        w, h = settings.waveshare_camera_resolution
        self._camera = WaveshareCamera(
            device_index=settings.waveshare_camera_device_index,
            width=w,
            height=h,
            fps=settings.waveshare_camera_fps,
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
        self._mjpeg_server: MjpegServer | None = (
            MjpegServer(settings.stream_port) if settings.stream_port else None
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
        """Verify all hardware components are operational then run until stopped."""

        # -- UGV controller --
        self._ugv.connect()
        logger.success("controller connected ✓")

        # Home pan-tilt before perception starts so camera begins centred.
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

        if self._mjpeg_server is not None:
            self._mjpeg_server.start()

        logger.info("Pipeline running. Press Ctrl+C to stop.")

        while True:
            self._update_lidar_state()
            cmd = self._decide_command()
            cmd = self._apply_mode_transition_stop(cmd)
            cmd = self._apply_estop_override(cmd)
            self._apply_motion_command(cmd)
            frame = self._camera.get_frame()
            results = self._run_inference(frame)
            bbox_u, bbox_v = self._extract_centroid(results)
            self._update_pan_state(bbox_u, bbox_v)
            self._push_stream_frame(self._annotate_frame(frame, results))
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
        """Apply one normalised motion command to the controller."""
        apply_motion_command(self._ugv, command)

    def _run_inference(self, frame: NDArray[np.uint8] | None) -> Any | None:
        """Run YOLO inference on *frame* and return the Results, or None.

        Parameters
        ----------
        frame : NDArray[np.uint8] | None
            Current camera frame, or ``None`` if no frame is available.

        Returns
        -------
        Any | None
            Ultralytics Results list, or ``None`` when inference is disabled
            or no frame is available.
        """
        if self._detector is None or frame is None:
            return None
        return self._detector.run(frame)

    def _extract_centroid(
        self, results: Any | None
    ) -> tuple[float | None, float | None]:
        """Return the ``(u, v)`` centroid of the highest-confidence detection.

        Parameters
        ----------
        results : Any | None
            Ultralytics Results list, or ``None`` when no inference was run.

        Returns
        -------
        tuple[float | None, float | None]
            Pixel centroid of the best detection, or ``(None, None)``.
        """
        if results is None:
            return None, None
        return select_target_centroid(results)

    def _annotate_frame(
        self,
        frame: NDArray[np.uint8] | None,
        results: Any | None,
    ) -> NDArray[np.uint8] | None:
        """Return a BGR frame with detection boxes overlaid, or the raw frame.

        Parameters
        ----------
        frame : NDArray[np.uint8] | None
            Raw camera frame.
        results : Any | None
            Ultralytics Results list, or ``None`` if inference was not run.

        Returns
        -------
        NDArray[np.uint8] | None
            Annotated frame if results are available, raw frame if not, or
            ``None`` when no frame was captured.
        """
        if frame is None:
            return None
        if results is not None:
            return results[0].plot()  # type: ignore[no-any-return]
        return frame

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

    def _push_stream_frame(self, frame: NDArray[np.uint8] | None) -> None:
        """Push *frame* to the MJPEG server if streaming is enabled.

        Parameters
        ----------
        frame : NDArray[np.uint8] | None
            Frame to stream, or ``None`` to skip (no frame available).
        """
        if self._mjpeg_server is not None and frame is not None:
            self._mjpeg_server.push_frame(frame)

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

    def _shutdown(self) -> None:
        """Release all hardware resources on exit."""
        logger.info("Shutting down pipeline...")
        if self._mjpeg_server is not None:
            try:
                self._mjpeg_server.stop()
            except Exception as exc:
                logger.warning(f"MJPEG server stop error: {exc}")
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
