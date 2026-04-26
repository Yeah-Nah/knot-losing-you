"""Angular offset calibration between the LDRobot D500 LiDAR and the Waveshare pan-tilt.

Measures ``delta_offset`` — the signed yaw angle (degrees) between the LiDAR's 0° forward
axis and the pan-tilt camera's mechanical zero — and writes it to ``sensor_config.yaml``
under the ``extrinsic`` key.

Must be run on the Raspberry Pi with the LiDAR and UGV rover connected.
Intrinsic calibration (``calibrate_waveshare_camera.py``) must have been completed first.

The ``ugv-follower`` package must be installed before running this script::

    pip install -e ugv-follower/

How to run
----------
::

    ugv-calibrate-angular --distance 2.0

    # Or via python -m:
    python -m tools.calibration.calibrate_angular_offset --distance 2.0

Arguments
---------
--distance       Approximate distance to the calibration target in metres (required).
--distance-tol   Half-width of the LiDAR range filter in metres (default: 0.2).
                 Only returns within [distance − tolerance, distance + tolerance] are kept.
--duration       Seconds to accumulate LiDAR data after the target is aligned (default: 5.0).
--camera-device  V4L2 device path for the Waveshare RGB camera (default: /dev/video0).
--sensor-config  Path to sensor_config.yaml to read and write (default:
                 configs/sensor_config.yaml relative to this script).

Hardware setup
--------------
1. Mount the UGV on a flat surface with clear space ahead (≥ 2 m recommended).
2. Connect the LDRobot D500 to the port listed under ``lidar.port`` in sensor_config.yaml.
3. Connect the UGV rover sub-controller to the port listed under ``ugv.port``.
4. Connect the Waveshare RGB camera (USB) — note its device path.
5. Power on the rover.

Calibration procedure
---------------------
1. Run the script with the desired ``--distance`` value.
2. The rover commands the pan-tilt to its mechanical zero (0°, 0°) and waits for the
   servo to settle.
3. A live MJPEG stream is served at ``http://<pi-ip>:8080/stream``.  Open this URL in a
   browser to see the camera feed with a vertical green guide line at column ``cx``
   (the principal point from intrinsic calibration — not the pixel-count midpoint).
4. Place a flat-faced cardboard box (≈ 20 cm wide) roughly ``--distance`` metres directly
   in front of the rover.  Physically slide it left or right until its face is bisected
   by the green line.
5. ``GET /confirm`` (e.g. ``curl http://<pi-ip>:8080/confirm``) to start the LiDAR scan.
   ``GET /abort`` to cancel without saving.
6. The script accumulates LiDAR returns for ``--duration`` seconds, keeping only returns
   in the forward ±20° arc at the expected distance.
7. The angular centroid of the cluster is computed as the median of the signed angles,
   avoiding wrap-around artefacts near 0°/360°.
8. Poll ``GET /status`` until ``state`` is ``COMPLETE`` (or ``FAILED``).
9. ``GET /save`` to write the result to sensor_config.yaml under ``extrinsic``.

Endpoints
---------
GET /stream   — MJPEG stream; open in browser to see the live annotated feed.
GET /status   — JSON: {"state": <name>, "delta_offset_deg": <float|null>,
                       "n_lidar_points": <int|null>, "error_message": <str|null>}.
GET /confirm  — Start LiDAR scan (only valid in WAITING_ALIGNMENT).
               Returns {"status": "scanning"} or 409 if in wrong state.
GET /abort    — Cancel any in-progress scan, transition to ABORTED, and shut down.
               Returns {"status": "aborted"}.
GET /save     — Write result to sensor_config.yaml (only valid in COMPLETE).
               Returns {"saved": true, "delta_offset_deg": <float>} or 409.
GET /reset    — Reset state back to WAITING_ALIGNMENT (valid from FAILED or COMPLETE).
               Returns {"status": "reset"} or 409 if called while SCANNING.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from enum import Enum
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml
from loguru import logger

from ugv_follower.control.ugv_controller import UGVController
from ugv_follower.perception.lidar_access import LidarAccess, LidarPoint
from ugv_follower.utils.camera_preflight import ensure_camera_device_available
from ugv_follower.utils.config_utils import get_project_root

_DEFAULT_SENSOR_CONFIG = get_project_root() / "configs" / "sensor_config.yaml"

# Half-width of the forward arc that is searched for LiDAR returns (degrees).
# Centred on 0°; covers [0, _FORWARD_ARC_DEG] and [360 - _FORWARD_ARC_DEG, 360).
_FORWARD_ARC_DEG: float = 20.0

# Minimum cluster size below which the result is flagged as noisy.
_MIN_CLUSTER_POINTS: int = 20

# Seconds to wait after set_pan_tilt(0, 0) for the servo to settle.
_SERVO_SETTLE_S: float = 1.5

# Seconds to drain stale serial-buffer data after lidar.start() before counting
# points.  The D500 streams continuously; data queued in the kernel USB serial
# buffer while the port was idle is discarded here so only fresh returns are used.
_LIDAR_WARMUP_S: float = 1.5

_STREAM_PORT: int = 8080
_JPEG_QUALITY: int = 80


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------


class CalibrationState(str, Enum):
    """State machine values for the calibration workflow.

    Using ``str`` as a mixin ensures ``state.value`` serialises to a plain
    string in JSON responses, preserving the existing endpoint contract.
    """

    WAITING_ALIGNMENT = "WAITING_ALIGNMENT"
    SCANNING = "SCANNING"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"
    ABORTED = "ABORTED"


# ---------------------------------------------------------------------------
# Data models
# ---------------------------------------------------------------------------


@dataclass
class CalibrationStatus:
    """Snapshot of calibration progress.

    Treated as an immutable value: ``CalibrationStateContainer`` replaces
    the whole instance atomically rather than mutating individual fields.
    """

    state: CalibrationState = CalibrationState.WAITING_ALIGNMENT
    delta_offset_deg: float | None = None
    n_lidar_points: int | None = None
    error_message: str | None = None


@dataclass(frozen=True)
class AngularOffsetConfig:
    """Validated, derived configuration for one calibration run.

    Constructed by ``_load_config()`` which reads both the YAML sensor config
    and the parsed CLI arguments, validates them, and derives convenience
    fields (``dist_min_mm``, ``dist_max_mm``).
    """

    cx: float
    cam_width: int
    cam_height: int
    lidar_port: str
    lidar_baud: int
    mounting_offset_deg: float
    ugv_port: str
    ugv_baud: int
    chassis_main: int
    chassis_module: int
    track_width: float
    target_distance_m: float
    distance_tol_m: float
    duration_s: float
    dist_min_mm: float
    dist_max_mm: float
    sensor_config_path: Path
    camera_device: str


# ---------------------------------------------------------------------------
# Thread-safe state container
# ---------------------------------------------------------------------------


class CalibrationStateContainer:
    """Thread-safe container for calibration progress and the latest annotated frame."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._status: CalibrationStatus = CalibrationStatus()
        self._annotated_frame: cv2.typing.MatLike | None = None

    def update_frame(self, annotated_frame: cv2.typing.MatLike) -> None:
        """Replace the stored annotated frame. Called from the frame thread."""
        with self._lock:
            self._annotated_frame = annotated_frame

    def get_annotated_jpeg(self) -> bytes | None:
        """JPEG-encode the latest annotated frame and return bytes, or None."""
        with self._lock:
            frame = self._annotated_frame
        if frame is None:
            return None
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, _JPEG_QUALITY])
        return bytes(buf) if ok else None

    def get_status(self) -> dict[str, Any]:
        """Return a JSON-serialisable snapshot of current state values."""
        with self._lock:
            s = self._status
        return {
            "state": s.state.value,
            "delta_offset_deg": s.delta_offset_deg,
            "n_lidar_points": s.n_lidar_points,
            "error_message": s.error_message,
        }

    def get_snapshot(self) -> CalibrationStatus:
        """Return a copy of the current ``CalibrationStatus``."""
        with self._lock:
            s = self._status
        return CalibrationStatus(
            state=s.state,
            delta_offset_deg=s.delta_offset_deg,
            n_lidar_points=s.n_lidar_points,
            error_message=s.error_message,
        )

    def try_start_scanning(self) -> bool:
        """Atomically transition WAITING_ALIGNMENT → SCANNING. Returns True on success."""
        with self._lock:
            if self._status.state != CalibrationState.WAITING_ALIGNMENT:
                return False
            self._status = CalibrationStatus(state=CalibrationState.SCANNING)
            return True

    def transition_to(self, new_status: CalibrationStatus) -> None:
        """Replace the current status atomically."""
        with self._lock:
            self._status = new_status

    def try_reset(self) -> bool:
        """Atomically reset to WAITING_ALIGNMENT, clearing previous results.

        Returns True on success, or False if the current state is SCANNING
        (a reset while scanning is not permitted).
        """
        with self._lock:
            if self._status.state == CalibrationState.SCANNING:
                return False
            self._status = CalibrationStatus()
            return True


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------


def _extract_cx(cfg: dict[str, Any]) -> float:
    """Return the horizontal principal point ``cx`` from ``waveshare_rgb.camera_matrix``.

    Parameters
    ----------
    cfg : dict[str, Any]
        Parsed sensor_config.yaml dict.

    Returns
    -------
    float
        ``cx`` in pixels.

    Raises
    ------
    ValueError
        If ``camera_matrix`` is ``null`` (intrinsic calibration not yet run).
    """
    matrix = cfg.get("waveshare_rgb", {}).get("camera_matrix")
    if matrix is None:
        raise ValueError(
            "waveshare_rgb.camera_matrix is null in sensor_config.yaml. "
            "Run calibrate_waveshare_camera.py first."
        )
    # camera_matrix is stored row-major: [[fx,0,cx],[0,fy,cy],[0,0,1]]
    return float(matrix[0][2])


def _in_forward_arc(angle_deg: float, arc: float = _FORWARD_ARC_DEG) -> bool:
    """Return True if *angle_deg* is within ±arc degrees of 0° (wrap-safe).

    Accepts angles in [0, arc] or [360 − arc, 360) without any modular
    arithmetic that could fail at the boundary.
    """
    return angle_deg <= arc or angle_deg >= (360.0 - arc)


def _to_signed(angle_deg: float) -> float:
    """Convert an angle in [0, 360) to a signed angle in (−180, +180].

    This prevents the wrap-around artefact that occurs when averaging angles
    straddling the 0°/360° boundary (e.g. 359° and 1° should average to 0°,
    not 180°).
    """
    return angle_deg if angle_deg <= 180.0 else angle_deg - 360.0


# ---------------------------------------------------------------------------
# Config parsing
# ---------------------------------------------------------------------------


def _load_config(cfg: dict[str, Any], args: argparse.Namespace) -> AngularOffsetConfig:
    """Parse and validate sensor config + CLI args into an ``AngularOffsetConfig``.

    Parameters
    ----------
    cfg : dict[str, Any]
        Parsed sensor_config.yaml dict.
    args : argparse.Namespace
        Parsed CLI arguments.

    Returns
    -------
    AngularOffsetConfig
        Validated, frozen configuration dataclass.

    Raises
    ------
    ValueError
        If required fields are missing, null, or out of range.
    KeyError
        If required top-level sections are absent from the sensor config.
    """
    target_distance_m: float = args.distance
    distance_tol_m: float = args.distance_tol
    duration_s: float = args.duration
    camera_device: str = args.camera_device
    sensor_config_path: Path = args.sensor_config.resolve()

    if target_distance_m <= 0:
        raise ValueError("--distance must be a positive number.")
    if target_distance_m < 1.0:
        logger.warning(
            f"Target distance {target_distance_m} m is less than 1 m. "
            "Sensor translation offset may introduce significant bearing error. "
            "Consider using a distance ≥ 1.5 m."
        )

    cx = _extract_cx(cfg)
    res = cfg.get("waveshare_rgb", {}).get("resolution", [1280, 720])
    cam_width, cam_height = int(res[0]), int(res[1])

    lidar_cfg: dict[str, Any] = cfg["lidar"]
    ugv_cfg: dict[str, Any] = cfg["ugv"]

    dist_min_mm = (target_distance_m - distance_tol_m) * 1000.0
    dist_max_mm = (target_distance_m + distance_tol_m) * 1000.0

    return AngularOffsetConfig(
        cx=cx,
        cam_width=cam_width,
        cam_height=cam_height,
        lidar_port=str(lidar_cfg["port"]),
        lidar_baud=int(lidar_cfg["baud_rate"]),
        mounting_offset_deg=float(lidar_cfg.get("mounting_offset_deg", 0.0)),
        ugv_port=str(ugv_cfg["port"]),
        ugv_baud=int(ugv_cfg["baud_rate"]),
        chassis_main=int(ugv_cfg["chassis_main"]),
        chassis_module=int(ugv_cfg["chassis_module"]),
        track_width=float(ugv_cfg["track_width"]),
        target_distance_m=target_distance_m,
        distance_tol_m=distance_tol_m,
        duration_s=duration_s,
        dist_min_mm=dist_min_mm,
        dist_max_mm=dist_max_mm,
        sensor_config_path=sensor_config_path,
        camera_device=camera_device,
    )


# ---------------------------------------------------------------------------
# Hardware interaction
# ---------------------------------------------------------------------------


def _accumulate_lidar(
    lidar: LidarAccess,
    duration_s: float,
    dist_min_mm: float,
    dist_max_mm: float,
    mounting_offset_deg: float,
    cancel_event: threading.Event,
) -> list[float]:
    """Collect LiDAR returns from the forward arc within the distance window.

    Runs for *duration_s* seconds (or until *cancel_event* is set), calling
    ``lidar.get_scan()`` in a tight loop.  Each returned packet yields up to
    12 points; only those in the forward arc and within the range window are
    kept.

    Parameters
    ----------
    lidar : LidarAccess
        Started ``LidarAccess`` instance.
    duration_s : float
        How long to accumulate (seconds).
    dist_min_mm : float
        Minimum accepted distance in millimetres.
    dist_max_mm : float
        Maximum accepted distance in millimetres.
    mounting_offset_deg : float
        Direction the LiDAR's own 0° axis points in the rover's reference
        frame (rover forward = 0°).  Raw LiDAR angles are converted to the
        rover frame via ``(lidar_angle + mounting_offset_deg) % 360`` before
        arc filtering and centroid computation.
    cancel_event : threading.Event
        When set, the accumulation loop exits early.

    Returns
    -------
    list[float]
        Signed angles (degrees, (−180, +180]) of all accepted returns,
        expressed in the rover's reference frame.
    """
    signed_angles: list[float] = []
    deadline = time.monotonic() + duration_s

    while time.monotonic() < deadline and not cancel_event.is_set():
        packet: list[LidarPoint] | None = lidar.get_scan()
        if packet is None:
            continue
        for pt in packet:
            dist = pt["distance"]
            angle = pt["angle"]
            if dist == 0:
                continue  # sentinel: out of range
            rover_angle = (angle + mounting_offset_deg) % 360.0
            if not _in_forward_arc(rover_angle):
                continue
            if not (dist_min_mm <= dist <= dist_max_mm):
                continue
            signed_angles.append(_to_signed(rover_angle))

    return signed_angles


# ---------------------------------------------------------------------------
# LiDAR scan thread
# ---------------------------------------------------------------------------


def _run_lidar_scan(
    lidar: LidarAccess,
    state: CalibrationStateContainer,
    duration_s: float,
    dist_min_mm: float,
    dist_max_mm: float,
    target_distance_m: float,
    distance_tol_m: float,
    mounting_offset_deg: float,
    cancel_event: threading.Event,
) -> None:
    """Accumulate LiDAR returns and transition state to COMPLETE or FAILED."""
    lidar.start()

    # Drain buffered/stale data before the measurement window begins.
    warmup_deadline = time.monotonic() + _LIDAR_WARMUP_S
    while time.monotonic() < warmup_deadline:
        lidar.get_scan()

    logger.info(f"LiDAR scan started ({duration_s} s)...")
    try:
        signed_angles = _accumulate_lidar(
            lidar,
            duration_s,
            dist_min_mm,
            dist_max_mm,
            mounting_offset_deg,
            cancel_event,
        )
    finally:
        lidar.stop()

    if cancel_event.is_set():
        logger.info("LiDAR scan cancelled.")
        return

    if len(signed_angles) == 0:
        logger.error(
            "No LiDAR returns matched the forward arc and distance filter. "
            f"Check that the target is within {target_distance_m} ± {distance_tol_m} m."
        )
        state.transition_to(
            CalibrationStatus(
                state=CalibrationState.FAILED,
                error_message="No LiDAR returns in forward arc within distance filter.",
            )
        )
        return

    n_pts = len(signed_angles)
    if n_pts < _MIN_CLUSTER_POINTS:
        logger.warning(
            f"Only {n_pts} LiDAR point(s) in cluster — fewer than {_MIN_CLUSTER_POINTS}. "
            "Result may be noisy. Consider increasing --duration or moving the target closer."
        )

    delta = float(np.median(signed_angles))
    logger.info(f"LiDAR scan complete: delta_offset_deg={delta:+.4f}°, n={n_pts}")
    state.transition_to(
        CalibrationStatus(
            state=CalibrationState.COMPLETE,
            delta_offset_deg=delta,
            n_lidar_points=n_pts,
        )
    )


# ---------------------------------------------------------------------------
# YAML write (atomic)
# ---------------------------------------------------------------------------


def _write_results_atomic(
    sensor_config_path: Path,
    delta_offset_deg: float,
    target_distance_m: float,
    n_lidar_points: int,
) -> None:
    """Merge the calibration result into sensor_config.yaml under ``extrinsic``.

    Reads the existing file, updates (or creates) the ``extrinsic`` key, and
    re-serialises via an atomic rename so a crash mid-write cannot corrupt the
    existing file.  All other top-level keys are preserved; comments and
    original formatting are not.

    Parameters
    ----------
    sensor_config_path : Path
        Path to ``sensor_config.yaml``.
    delta_offset_deg : float
        Measured angular offset in signed degrees (−180, +180].
    target_distance_m : float
        Target distance used during calibration, in metres.
    n_lidar_points : int
        Number of LiDAR cluster points used to compute the median.
    """
    with sensor_config_path.open() as f:
        config: dict[str, Any] = yaml.safe_load(f) or {}

    # Preserve any existing extrinsic fields (e.g. a future full T_C^L transform)
    # and update only the angular-offset calibration keys.
    extrinsic = config.get("extrinsic")
    if not isinstance(extrinsic, dict):
        extrinsic = {}
    config["extrinsic"] = extrinsic
    extrinsic.update(
        {
            "lidar_to_pantilt_offset_deg": round(float(delta_offset_deg), 4),
            "calibration_method": "angular_offset",
            "target_distance_m": round(float(target_distance_m), 3),
            "n_lidar_points": int(n_lidar_points),
        }
    )

    tmp_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            dir=sensor_config_path.parent,
            suffix=".tmp",
            delete=False,
        ) as tmp:
            yaml.dump(config, tmp, default_flow_style=False, sort_keys=False)
            tmp_path = Path(tmp.name)
        os.replace(tmp_path, sensor_config_path)
        tmp_path = None  # os.replace succeeded; no cleanup needed
    finally:
        if tmp_path is not None:
            tmp_path.unlink(missing_ok=True)

    logger.info(f"Results written to {sensor_config_path}")


# ---------------------------------------------------------------------------
# Calibration orchestrator
# ---------------------------------------------------------------------------


class CalibrationOrchestrator:
    """Owns calibration logic: starting scans and persisting results.

    Decoupled from HTTP transport — has no knowledge of request/response
    handling.

    Parameters
    ----------
    config : AngularOffsetConfig
        Validated, frozen configuration for this calibration run.
    lidar : LidarAccess
        Initialised (but not yet started) LiDAR access object.
    state : CalibrationStateContainer
        Shared thread-safe state container.
    """

    def __init__(
        self,
        config: AngularOffsetConfig,
        lidar: LidarAccess,
        state: CalibrationStateContainer,
    ) -> None:
        self._config = config
        self._lidar = lidar
        self._state = state
        self._cancel_event: threading.Event = threading.Event()

    def start_scan(self) -> None:
        """Transition to SCANNING and spawn the LiDAR accumulation thread.

        Raises
        ------
        RuntimeError
            If not currently in WAITING_ALIGNMENT state.
        """
        if not self._state.try_start_scanning():
            raise RuntimeError("not in WAITING_ALIGNMENT state")
        self._cancel_event.clear()
        threading.Thread(
            target=_run_lidar_scan,
            args=(
                self._lidar,
                self._state,
                self._config.duration_s,
                self._config.dist_min_mm,
                self._config.dist_max_mm,
                self._config.target_distance_m,
                self._config.distance_tol_m,
                self._config.mounting_offset_deg,
                self._cancel_event,
            ),
            daemon=True,
        ).start()

    def cancel_scan(self) -> None:
        """Signal the running LiDAR scan thread (if any) to stop early."""
        self._cancel_event.set()

    def save_results(self) -> float:
        """Write calibration results to sensor_config.yaml.

        Returns
        -------
        float
            The saved ``delta_offset_deg`` value.

        Raises
        ------
        RuntimeError
            If not currently in COMPLETE state.
        """
        snapshot = self._state.get_snapshot()
        if snapshot.state != CalibrationState.COMPLETE:
            raise RuntimeError("not in COMPLETE state")
        delta = snapshot.delta_offset_deg
        n_pts = snapshot.n_lidar_points
        if delta is None or n_pts is None:
            raise RuntimeError("COMPLETE state missing result fields — this is a bug")
        _write_results_atomic(
            self._config.sensor_config_path,
            delta,
            self._config.target_distance_m,
            n_pts,
        )
        return delta


# ---------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------


def _make_handler(
    state: CalibrationStateContainer,
    server_ref: list[ThreadingHTTPServer | None],
    orchestrator: CalibrationOrchestrator,
) -> type[BaseHTTPRequestHandler]:
    """Return a handler class closed over the shared state and orchestrator."""

    class Handler(BaseHTTPRequestHandler):
        def log_message(
            self, format: str, *args: object
        ) -> None:  # silence access log  # noqa: A002
            pass

        def do_GET(self) -> None:  # noqa: N802
            if self.path == "/stream":
                self._serve_stream()
            elif self.path == "/status":
                self._serve_json(state.get_status())
            elif self.path == "/confirm":
                self._handle_confirm()
            elif self.path == "/abort":
                self._handle_abort()
            elif self.path == "/save":
                self._handle_save()
            elif self.path == "/reset":
                self._handle_reset()
            else:
                self.send_error(404)

        def _serve_json(self, data: dict[str, Any], status: int = 200) -> None:
            body = json.dumps(data).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _serve_stream(self) -> None:
            self.send_response(200)
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame",
            )
            self.end_headers()
            try:
                while True:
                    jpeg = state.get_annotated_jpeg()
                    if jpeg is None:
                        time.sleep(0.05)
                        continue
                    part = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                        + jpeg
                        + b"\r\n"
                    )
                    self.wfile.write(part)
                    self.wfile.flush()
                    time.sleep(1 / 30)
            except (BrokenPipeError, ConnectionResetError):
                pass  # client disconnected

        def _handle_confirm(self) -> None:
            try:
                orchestrator.start_scan()
            except RuntimeError:
                self._serve_json({"error": "not in WAITING_ALIGNMENT state"}, 409)
                return
            self._serve_json({"status": "scanning"})

        def _handle_abort(self) -> None:
            orchestrator.cancel_scan()
            state.transition_to(CalibrationStatus(state=CalibrationState.ABORTED))
            srv = server_ref[0]
            if srv is not None:
                # server.shutdown() blocks until serve_forever() returns; run it in a
                # daemon thread so this handler can return its response first.
                threading.Thread(target=srv.shutdown, daemon=True).start()
            self._serve_json({"status": "aborted"})

        def _handle_reset(self) -> None:
            if not state.try_reset():
                self._serve_json({"error": "cannot reset while SCANNING"}, 409)
                return
            self._serve_json({"status": "reset"})

        def _handle_save(self) -> None:
            try:
                delta = orchestrator.save_results()
            except RuntimeError:
                self._serve_json({"error": "not in COMPLETE state"}, 409)
                return
            self._serve_json({"saved": True, "delta_offset_deg": delta})

    return Handler


# ---------------------------------------------------------------------------
# Frame thread
# ---------------------------------------------------------------------------


def _run_frame(
    cap: cv2.VideoCapture,
    cx: float,
    state: CalibrationStateContainer,
    stop_event: threading.Event,
) -> None:
    """Read frames, draw the guide overlay, and push annotated frames to shared state."""
    cx_col = int(round(cx))
    while not stop_event.is_set():
        ok, frame = cap.read()
        if not ok or frame is None:
            logger.warning("Camera returned no frame — retrying...")
            time.sleep(0.05)
            continue

        h = frame.shape[0]
        annotated = frame.copy()
        cv2.line(annotated, (cx_col, 0), (cx_col, h - 1), (0, 255, 0), 2)
        cv2.putText(
            annotated,
            "Centre target on green line, then GET /confirm",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            f"cx = {cx:.1f} px",
            (10, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
        state.update_frame(annotated)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _build_arg_parser() -> argparse.ArgumentParser:
    """Build and return the CLI argument parser."""
    parser = argparse.ArgumentParser(
        description=(
            "Measure the yaw angular offset between the LiDAR forward axis "
            "and the pan-tilt mechanical zero."
        )
    )
    parser.add_argument(
        "--distance",
        required=True,
        type=float,
        metavar="METRES",
        help="Approximate distance to the calibration target in metres.",
    )
    parser.add_argument(
        "--distance-tol",
        type=float,
        default=0.2,
        metavar="METRES",
        help="Half-width of the LiDAR range filter in metres (default: 0.2).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        metavar="SECONDS",
        help="Seconds to accumulate LiDAR data after target alignment (default: 5.0).",
    )
    parser.add_argument(
        "--camera-device",
        type=str,
        default="/dev/video0",
        metavar="PATH",
        help="V4L2 device path for the Waveshare RGB camera (default: /dev/video0).",
    )
    parser.add_argument(
        "--sensor-config",
        type=Path,
        default=_DEFAULT_SENSOR_CONFIG,
        metavar="PATH",
        help=f"Path to sensor_config.yaml (default: {_DEFAULT_SENSOR_CONFIG}).",
    )
    return parser


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    args = _build_arg_parser().parse_args()

    sensor_config_path: Path = args.sensor_config.resolve()
    if not sensor_config_path.exists():
        logger.error(f"Sensor config not found: {sensor_config_path}")
        sys.exit(1)

    with sensor_config_path.open() as f:
        cfg: dict[str, Any] = yaml.safe_load(f) or {}

    try:
        config = _load_config(cfg, args)
    except (ValueError, KeyError) as exc:
        logger.error(str(exc))
        sys.exit(1)

    logger.info(f"Loaded cx = {config.cx:.2f} px from {sensor_config_path}")
    logger.info(
        f"Target distance: {config.target_distance_m} m  "
        f"(range filter: [{config.dist_min_mm:.0f}, {config.dist_max_mm:.0f}] mm)"
    )
    logger.info(
        f"LiDAR mounting offset: {config.mounting_offset_deg:+.1f}° (rover frame)"
    )

    # -- Initialise hardware ---------------------------------------------------
    ugv = UGVController(
        port=config.ugv_port,
        baud_rate=config.ugv_baud,
        chassis_main=config.chassis_main,
        chassis_module=config.chassis_module,
        track_width=config.track_width,
    )
    lidar = LidarAccess(port=config.lidar_port, baud_rate=config.lidar_baud)
    cap: cv2.VideoCapture | None = None
    frame_thread: threading.Thread | None = None
    server: ThreadingHTTPServer | None = None
    server_ref: list[ThreadingHTTPServer | None] = [
        None
    ]  # mutable box — populated after server is constructed
    state = CalibrationStateContainer()
    stop_event: threading.Event | None = None

    try:
        # -- Connect hardware and zero the pan-tilt ----------------------------
        ugv.connect()
        ugv.set_pan_tilt(0.0, 0.0)
        logger.info(
            f"Pan-tilt commanded to (0°, 0°). Waiting {_SERVO_SETTLE_S} s to settle..."
        )
        time.sleep(_SERVO_SETTLE_S)

        try:
            ensure_camera_device_available(config.camera_device)
        except RuntimeError as exc:
            logger.error(str(exc))
            sys.exit(1)

        # -- Open camera -------------------------------------------------------
        cap = cv2.VideoCapture(config.camera_device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.cam_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.cam_height)
        cap.set(
            cv2.CAP_PROP_AUTO_EXPOSURE, 3
        )  # 3 = aperture priority (auto); clears any stale manual value
        if not cap.isOpened():
            logger.error(
                f"Could not open camera device {config.camera_device}. "
                "Try a different --camera-device (e.g. /dev/video0)."
            )
            sys.exit(1)

        # -- Start frame thread ------------------------------------------------
        stop_event = threading.Event()
        frame_thread = threading.Thread(
            target=_run_frame,
            args=(cap, config.cx, state, stop_event),
            daemon=True,
        )
        frame_thread.start()
        logger.info("Frame thread started.")

        # -- Start HTTP server -------------------------------------------------
        orchestrator = CalibrationOrchestrator(config, lidar, state)
        server = ThreadingHTTPServer(
            ("0.0.0.0", _STREAM_PORT),
            _make_handler(state, server_ref, orchestrator),
        )
        server_ref[0] = server  # bind into handler closure after construction

        logger.info(
            f"HTTP server running on port {_STREAM_PORT}. "
            f"Open http://<pi-ip>:{_STREAM_PORT}/stream in your browser."
        )
        logger.info(
            "Endpoints: /stream (MJPEG), /status (JSON), "
            "/confirm (start scan), /abort, /save, /reset"
        )

        try:
            server.serve_forever()
        except KeyboardInterrupt:
            logger.info("Shutting down...")

    finally:
        # -- Teardown ----------------------------------------------------------
        if stop_event is not None:
            stop_event.set()
        if frame_thread is not None:
            frame_thread.join(timeout=2)
            if frame_thread.is_alive():
                logger.warning("Frame thread did not stop within 2 s.")
        if cap is not None:
            cap.set(
                cv2.CAP_PROP_AUTO_EXPOSURE, 3
            )  # restore auto-exposure before release
            cap.release()
        if server is not None:
            server.server_close()
        lidar.stop()
        ugv.disconnect()


if __name__ == "__main__":
    main()
