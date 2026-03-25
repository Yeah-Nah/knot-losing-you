"""Angular offset calibration between the LDRobot D500 LiDAR and the Waveshare pan-tilt.

Measures ``delta_offset`` — the signed yaw angle (degrees) between the LiDAR's 0° forward
axis and the pan-tilt camera's mechanical zero — and writes it to ``sensor_config.yaml``
under the ``extrinsic`` key.

Must be run on the Raspberry Pi with the LiDAR and UGV rover connected.
Intrinsic calibration (``calibrate_waveshare_camera.py``) must have been completed first.

How to run
----------
::

    python calibrate_angular_offset.py --distance 2.0

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
   in the forward ±30° arc at the expected distance.
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
GET /abort    — Transition to ABORTED and shut down the server.
               Returns {"status": "aborted"}.
GET /save     — Write result to sensor_config.yaml (only valid in COMPLETE).
               Returns {"saved": true, "delta_offset_deg": <float>} or 409.
GET /reset    — Reset state back to WAITING_ALIGNMENT (valid from FAILED or COMPLETE).
               Returns {"status": "reset"} or 409 if called while SCANNING.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
import yaml
from loguru import logger

_SCRIPT_DIR = Path(__file__).resolve().parent
_DEFAULT_SENSOR_CONFIG = _SCRIPT_DIR / "configs" / "sensor_config.yaml"

# Add the ugv-follower directory to sys.path so src.* imports resolve when
# the package is not installed (e.g. running directly from the repo).
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from src.control.ugv_controller import UGVController  # noqa: E402
from src.perception.lidar_access import LidarAccess, LidarPoint  # noqa: E402

# Half-width of the forward arc that is searched for LiDAR returns (degrees).
# Centred on 0°; covers [0, _FORWARD_ARC_DEG] and [360 - _FORWARD_ARC_DEG, 360).
_FORWARD_ARC_DEG: float = 30.0

# Minimum cluster size below which the result is flagged as noisy.
_MIN_CLUSTER_POINTS: int = 20

# Seconds to wait after set_pan_tilt(0, 0) for the servo to settle.
_SERVO_SETTLE_S: float = 1.5

_STREAM_PORT: int = 8080
_JPEG_QUALITY: int = 80

_STATE_WAITING_ALIGNMENT = "WAITING_ALIGNMENT"
_STATE_SCANNING          = "SCANNING"
_STATE_COMPLETE          = "COMPLETE"
_STATE_FAILED            = "FAILED"
_STATE_ABORTED           = "ABORTED"


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------


def _extract_cx(cfg: dict) -> float:
    """Return the horizontal principal point ``cx`` from ``waveshare_rgb.camera_matrix``.

    Parameters
    ----------
    cfg:
        Parsed sensor_config.yaml dict.

    Returns
    -------
    float
        ``cx`` in pixels.

    Raises
    ------
    SystemExit
        If ``camera_matrix`` is ``null`` (intrinsic calibration not yet run).
    """
    matrix = cfg.get("waveshare_rgb", {}).get("camera_matrix")
    if matrix is None:
        logger.error(
            "waveshare_rgb.camera_matrix is null in sensor_config.yaml. "
            "Run calibrate_waveshare_camera.py first."
        )
        sys.exit(1)
    # camera_matrix is stored row-major: [[fx,0,cx],[0,fy,cy],[0,0,1]]
    cx: float = float(matrix[0][2])
    return cx


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
# Hardware interaction
# ---------------------------------------------------------------------------


def _accumulate_lidar(
    lidar: LidarAccess,
    duration_s: float,
    dist_min_mm: float,
    dist_max_mm: float,
) -> list[float]:
    """Collect LiDAR returns from the forward arc within the distance window.

    Runs for *duration_s* seconds, calling ``lidar.get_scan()`` in a tight
    loop.  Each returned packet yields up to 12 points; only those in the
    forward arc and within the range window are kept.

    Parameters
    ----------
    lidar:
        Started ``LidarAccess`` instance.
    duration_s:
        How long to accumulate (seconds).
    dist_min_mm:
        Minimum accepted distance in millimetres.
    dist_max_mm:
        Maximum accepted distance in millimetres.

    Returns
    -------
    list[float]
        Signed angles (degrees, (−180, +180]) of all accepted returns.
    """
    signed_angles: list[float] = []
    deadline = time.monotonic() + duration_s

    while time.monotonic() < deadline:
        packet: list[LidarPoint] | None = lidar.get_scan()
        if packet is None:
            continue
        for pt in packet:
            dist = pt["distance"]
            angle = pt["angle"]
            if dist == 0:
                continue  # sentinel: out of range
            if not _in_forward_arc(angle):
                continue
            if not (dist_min_mm <= dist <= dist_max_mm):
                continue
            signed_angles.append(_to_signed(angle))

    return signed_angles


# ---------------------------------------------------------------------------
# Shared calibration state
# ---------------------------------------------------------------------------


class _CalibrationState:
    """Thread-safe container for calibration progress and the latest annotated frame."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.state: str = _STATE_WAITING_ALIGNMENT
        self.delta_offset_deg: float | None = None
        self.n_lidar_points: int | None = None
        self.error_message: str | None = None
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

    def get_status(self) -> dict:
        """Return a snapshot dict of current state values."""
        with self._lock:
            return {
                "state": self.state,
                "delta_offset_deg": self.delta_offset_deg,
                "n_lidar_points": self.n_lidar_points,
                "error_message": self.error_message,
            }

    def try_start_scanning(self) -> bool:
        """Atomically transition WAITING_ALIGNMENT -> SCANNING. Returns True on success."""
        with self._lock:
            if self.state != _STATE_WAITING_ALIGNMENT:
                return False
            self.state = _STATE_SCANNING
            return True

    def transition_to(self, new_state: str, **kwargs: object) -> None:
        """Atomically set the state and any provided keyword fields."""
        with self._lock:
            self.state = new_state
            for key, value in kwargs.items():
                setattr(self, key, value)


# ---------------------------------------------------------------------------
# Frame thread
# ---------------------------------------------------------------------------


def _run_frame(
    cap: cv2.VideoCapture,
    cx: float,
    state: _CalibrationState,
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
# LiDAR scan thread
# ---------------------------------------------------------------------------


def _run_lidar_scan(
    lidar: LidarAccess,
    state: _CalibrationState,
    duration_s: float,
    dist_min_mm: float,
    dist_max_mm: float,
    target_distance_m: float,
    distance_tol_m: float,
) -> None:
    """Accumulate LiDAR returns and transition state to COMPLETE or FAILED."""
    logger.info(f"LiDAR scan started ({duration_s} s)...")
    signed_angles = _accumulate_lidar(lidar, duration_s, dist_min_mm, dist_max_mm)

    if len(signed_angles) == 0:
        logger.error(
            "No LiDAR returns matched the forward arc and distance filter. "
            f"Check that the target is within {target_distance_m} ± {distance_tol_m} m."
        )
        state.transition_to(
            _STATE_FAILED,
            error_message="No LiDAR returns in forward arc within distance filter.",
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
    state.transition_to(_STATE_COMPLETE, delta_offset_deg=delta, n_lidar_points=n_pts)


# ---------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------


def _make_handler(
    state: _CalibrationState,
    server_ref: list,
    lidar: LidarAccess,
    duration_s: float,
    dist_min_mm: float,
    dist_max_mm: float,
    target_distance_m: float,
    distance_tol_m: float,
    sensor_config_path: Path,
) -> type[BaseHTTPRequestHandler]:
    """Return a handler class closed over the shared state and hardware references."""

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _fmt: str, *_args: object) -> None:  # silence access log
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

        def _serve_json(self, data: dict, status: int = 200) -> None:
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
            if not state.try_start_scanning():
                self._serve_json({"error": "not in WAITING_ALIGNMENT state"}, 409)
                return
            threading.Thread(
                target=_run_lidar_scan,
                args=(
                    lidar, state, duration_s,
                    dist_min_mm, dist_max_mm,
                    target_distance_m, distance_tol_m,
                ),
                daemon=True,
            ).start()
            self._serve_json({"status": "scanning"})

        def _handle_abort(self) -> None:
            state.transition_to(_STATE_ABORTED)
            srv = server_ref[0]
            if srv is not None:
                # server.shutdown() blocks until serve_forever() returns; run it in a
                # daemon thread so this handler can return its response first.
                threading.Thread(target=srv.shutdown, daemon=True).start()
            self._serve_json({"status": "aborted"})

        def _handle_reset(self) -> None:
            with state._lock:
                if state.state == _STATE_SCANNING:
                    self._serve_json({"error": "cannot reset while SCANNING"}, 409)
                    return
                state.state = _STATE_WAITING_ALIGNMENT
                state.delta_offset_deg = None
                state.n_lidar_points = None
                state.error_message = None
            self._serve_json({"status": "reset"})

        def _handle_save(self) -> None:
            status = state.get_status()
            if status["state"] != _STATE_COMPLETE:
                self._serve_json({"error": "not in COMPLETE state"}, 409)
                return
            _write_results(
                sensor_config_path,
                status["delta_offset_deg"],
                target_distance_m,
                status["n_lidar_points"],
            )
            self._serve_json({
                "saved": True,
                "delta_offset_deg": status["delta_offset_deg"],
            })

    return Handler


# ---------------------------------------------------------------------------
# YAML write
# ---------------------------------------------------------------------------


def _write_results(
    sensor_config_path: Path,
    delta_offset_deg: float,
    target_distance_m: float,
    n_lidar_points: int,
) -> None:
    """Merge the calibration result into sensor_config.yaml under ``extrinsic``.

    Reads the existing file, updates (or creates) the ``extrinsic`` key, and
    re-serialises the full configuration.  All other top-level keys are
    preserved; comments and original formatting are not.

    Parameters
    ----------
    sensor_config_path:
        Path to ``sensor_config.yaml``.
    delta_offset_deg:
        Measured angular offset in signed degrees (−180, +180].
    target_distance_m:
        Target distance used during calibration, in metres.
    n_lidar_points:
        Number of LiDAR cluster points used to compute the median.
    """
    with sensor_config_path.open() as f:
        config: dict = yaml.safe_load(f) or {}

    config["extrinsic"] = {
        "lidar_to_pantilt_offset_deg": round(float(delta_offset_deg), 4),
        "calibration_method": "angular_offset",
        "target_distance_m": round(float(target_distance_m), 3),
        "n_lidar_points": int(n_lidar_points),
    }

    with sensor_config_path.open("w") as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    logger.info(f"Results written to {sensor_config_path}")


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

    target_distance_m: float = args.distance
    distance_tol_m: float = args.distance_tol
    duration_s: float = args.duration
    camera_device: str = args.camera_device
    sensor_config_path: Path = args.sensor_config.resolve()

    # -- Validate inputs -------------------------------------------------------
    if target_distance_m <= 0:
        logger.error("--distance must be a positive number.")
        sys.exit(1)
    if target_distance_m < 1.0:
        logger.warning(
            f"Target distance {target_distance_m} m is less than 1 m. "
            "Sensor translation offset may introduce significant bearing error. "
            "Consider using a distance ≥ 1.5 m."
        )
    if not sensor_config_path.exists():
        logger.error(f"Sensor config not found: {sensor_config_path}")
        sys.exit(1)

    # -- Load config -----------------------------------------------------------
    with sensor_config_path.open() as f:
        cfg: dict = yaml.safe_load(f) or {}

    cx = _extract_cx(cfg)
    res = cfg.get("waveshare_rgb", {}).get("resolution", [1280, 720])
    cam_width, cam_height = int(res[0]), int(res[1])
    logger.info(f"Loaded cx = {cx:.2f} px from {sensor_config_path}")

    lidar_port: str = cfg["lidar"]["port"]
    lidar_baud: int = cfg["lidar"]["baud_rate"]
    ugv_port: str = cfg["ugv"]["port"]
    ugv_baud: int = cfg["ugv"]["baud_rate"]
    chassis_main: int = cfg["ugv"]["chassis_main"]
    chassis_module: int = cfg["ugv"]["chassis_module"]
    track_width: float = cfg["ugv"]["track_width"]

    dist_min_mm = (target_distance_m - distance_tol_m) * 1000.0
    dist_max_mm = (target_distance_m + distance_tol_m) * 1000.0

    logger.info(
        f"Target distance: {target_distance_m} m  "
        f"(range filter: [{dist_min_mm:.0f}, {dist_max_mm:.0f}] mm)"
    )

    # -- Initialise hardware ---------------------------------------------------
    ugv = UGVController(
        port=ugv_port,
        baud_rate=ugv_baud,
        chassis_main=chassis_main,
        chassis_module=chassis_module,
        track_width=track_width,
    )
    lidar = LidarAccess(port=lidar_port, baud_rate=lidar_baud)
    cap: cv2.VideoCapture | None = None
    frame_thread: threading.Thread | None = None
    server: ThreadingHTTPServer | None = None
    server_ref: list = [None]  # mutable box — populated after server is constructed

    try:
        # -- Connect hardware and zero the pan-tilt ----------------------------
        ugv.connect()
        ugv.set_pan_tilt(0.0, 0.0)
        logger.info(f"Pan-tilt commanded to (0°, 0°). Waiting {_SERVO_SETTLE_S} s to settle...")
        time.sleep(_SERVO_SETTLE_S)

        lidar.start()

        # -- Open camera -------------------------------------------------------
        cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_height)
        if not cap.isOpened():
            logger.error(
                f"Could not open camera device {camera_device}. "
                "Try a different --camera-device (e.g. /dev/video0)."
            )
            sys.exit(1)

        # -- Start frame thread ------------------------------------------------
        state = _CalibrationState()
        stop_event = threading.Event()

        frame_thread = threading.Thread(
            target=_run_frame,
            args=(cap, cx, state, stop_event),
            daemon=True,
        )
        frame_thread.start()
        logger.info("Frame thread started.")

        # -- Start HTTP server -------------------------------------------------
        server = ThreadingHTTPServer(
            ("0.0.0.0", _STREAM_PORT),
            _make_handler(
                state, server_ref, lidar, duration_s,
                dist_min_mm, dist_max_mm,
                target_distance_m, distance_tol_m,
                sensor_config_path,
            ),
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
        stop_event.set()
        if frame_thread is not None:
            frame_thread.join(timeout=2)
        if cap is not None:
            cap.release()
        if server is not None:
            server.server_close()
        lidar.stop()
        ugv.disconnect()


if __name__ == "__main__":
    main()
