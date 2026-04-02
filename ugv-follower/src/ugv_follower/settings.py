"""UGV follower settings — loads and validates all YAML config files."""

from __future__ import annotations

import os
import sys
from pathlib import Path

from loguru import logger

from .utils.config_utils import get_project_root, load_yaml

from typing import Any


class Settings:
    """Loads and validates pipeline, model, and sensor configuration.

    Parameters
    ----------
    pipeline_config_path : str | Path
        Path to ``pipeline_config.yaml``.
    model_config_path : str | Path
        Path to ``model_config.yaml``.
    sensor_config_path : str | Path
        Path to ``sensor_config.yaml``.
    """

    def __init__(
        self,
        pipeline_config_path: str | Path,
        model_config_path: str | Path,
        sensor_config_path: str | Path,
    ) -> None:
        self._root: Path = get_project_root()

        self.pipeline_config: dict[str, Any] = load_yaml(pipeline_config_path)
        self.model_config: dict[str, Any] = load_yaml(model_config_path)
        self.sensor_config: dict[str, Any] = load_yaml(sensor_config_path)

        self.output_dir: Path = self._resolve_output_dir()
        self.model_path: Path = self._resolve_model_path()

        self._validate()

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _resolve_output_dir(self) -> Path:
        raw = str(self.pipeline_config.get("output_dir", "output/"))
        return (self._root / raw).resolve()

    def _resolve_model_path(self) -> Path:
        model_filename = str(self.model_config.get("model", ""))
        return (self._root / "models" / model_filename).resolve()

    def _has_display(self) -> bool:
        """Return True if a display server is available for GUI output."""
        if sys.platform == "linux":
            return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        return True

    def _validate(self) -> None:
        if self.inference_enabled:
            model_name = str(self.model_config.get("model", "")).strip()
            if not model_name:
                logger.error(
                    "Inference is enabled but no model filename is configured. "
                    "Set the 'model' key in model_config.yaml to a valid model file."
                )
                raise ValueError(
                    "Model filename must be set in model_config.yaml when inference_enabled is True."
                )
            if not self.model_path.is_file():
                logger.error(f"Model file not found: {self.model_path}")
                raise FileNotFoundError(f"Model file not found: {self.model_path}")
        if self.live_view_enabled and not self._has_display():
            logger.error(
                "live_view_enabled is True but no display detected. "
                "Set 'live_view_enabled: False' in pipeline_config.yaml for headless operation."
            )
            raise OSError(
                "No display detected. Set 'live_view_enabled: False' in pipeline_config.yaml."
            )
        logger.debug(f"Settings validated. Project root: {self._root}")

    # ------------------------------------------------------------------
    # Pipeline config properties
    # ------------------------------------------------------------------

    @property
    def inference_enabled(self) -> bool:
        """Whether to run YOLO inference on each frame."""
        return bool(self.pipeline_config.get("inference_enabled", False))

    @property
    def live_view_enabled(self) -> bool:
        """Whether to display the camera feed in real-time."""
        return bool(self.pipeline_config.get("live_view_enabled", True))

    @property
    def recording_enabled(self) -> bool:
        """Whether to record the camera feed to disk."""
        return bool(self.pipeline_config.get("recording_enabled", False))

    @property
    def dev_or_pi(self) -> str:
        """Target runtime environment: ``'dev'`` or ``'pi'``."""
        return str(self.pipeline_config.get("dev_or_pi", "dev"))

    # ------------------------------------------------------------------
    # Sensor config properties
    # ------------------------------------------------------------------

    @property
    def _ugv_cfg(self) -> dict[str, Any]:
        return dict(self.sensor_config.get("ugv", {}))

    @property
    def ugv_port(self) -> str:
        """Serial port for the UGV sub-controller."""
        return str(self._ugv_cfg.get("port", "/dev/ttyAMA0"))

    @property
    def ugv_baud_rate(self) -> int:
        """Baud rate for the UGV serial connection."""
        return int(self._ugv_cfg.get("baud_rate", 115200))

    @property
    def ugv_chassis_main(self) -> int:
        """Chassis type code sent to the UGV sub-controller on connect."""
        return int(self._ugv_cfg.get("chassis_main", 2))

    @property
    def ugv_chassis_module(self) -> int:
        """Module type code sent to the UGV sub-controller on connect."""
        return int(self._ugv_cfg.get("chassis_module", 0))

    @property
    def ugv_track_width(self) -> float:
        """Wheel centre-to-centre distance in metres."""
        return float(self._ugv_cfg.get("track_width", 0.3))

    @property
    def _camera_cfg(self) -> dict[str, Any]:
        return dict(self.sensor_config.get("camera", {}))

    @property
    def camera_fps(self) -> int:
        """Target frame rate for the OAK-D colour camera."""
        return int(self._camera_cfg.get("fps", 30))

    @property
    def camera_resolution(self) -> tuple[int, int]:
        """OAK-D colour camera output resolution as (width, height)."""
        res = self._camera_cfg.get("colour_resolution", [1920, 1080])
        return (int(res[0]), int(res[1]))

    @property
    def _lidar_cfg(self) -> dict[str, Any]:
        return dict(self.sensor_config.get("lidar", {}))

    @property
    def lidar_port(self) -> str:
        """Serial port for the LiDAR sensor."""
        return str(self._lidar_cfg.get("port", "/dev/ttyUSB0"))

    @property
    def lidar_baud_rate(self) -> int:
        """Baud rate for the LiDAR serial connection."""
        return int(self._lidar_cfg.get("baud_rate", 230400))

    # ------------------------------------------------------------------
    # Pan-tilt servo calibration properties
    # ------------------------------------------------------------------

    @property
    def _pan_tilt_servo_cfg(self) -> dict[str, Any]:
        return dict(self.sensor_config.get("pan_tilt_servo") or {})

    @property
    def pan_tilt_servo_angle_model(self) -> str | None:
        """Angle measurement model recorded during calibration, or None if uncalibrated."""
        v = self._pan_tilt_servo_cfg.get("angle_measurement_model")
        return str(v) if v is not None else None

    @property
    def pan_tilt_servo_camera_forward_offset_m(self) -> float | None:
        """Forward offset of camera from pan-tilt rotation centre (metres), or None."""
        v = self._pan_tilt_servo_cfg.get("camera_forward_offset_m")
        return float(v) if v is not None else None

    @property
    def pan_tilt_servo_calibration_target_distance_m(self) -> float | None:
        """Distance from pan-tilt rotation centre to calibration target (metres), or None."""
        v = self._pan_tilt_servo_cfg.get("calibration_target_distance_m")
        return float(v) if v is not None else None
