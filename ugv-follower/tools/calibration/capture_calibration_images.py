"""Headless calibration image capture for the Waveshare RGB camera.

Runs on the Pi. Serves an MJPEG stream on http://<pi-ip>:8080/stream so the
operator can watch the live feed (with checkerboard corners overlaid) from a
browser on their laptop. Two HTTP endpoints let the operator trigger captures
and check progress without needing a local display.

Endpoints
---------
GET /stream   — MJPEG stream; open in browser to see the live annotated feed.
GET /capture  — Save the current raw frame to calibration/images/ if corners are
                currently detected. Returns JSON {"saved": bool, "count": int}.
GET /status   — Returns JSON {"count": int, "corners_visible": bool}.

Usage
-----
::

    ugv-capture-calibration

    # Or via python -m:
    python -m tools.calibration.capture_calibration_images

The script reads configs/calibration_config.yaml for the board dimensions,
camera device index, and UGV serial port. It zeros the pan-tilt servo before
opening the camera, then starts the HTTP server.

Press Ctrl-C to stop.
"""

from __future__ import annotations

import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import yaml
from loguru import logger

from ugv_follower.control.ugv_controller import UGVController
from ugv_follower.utils.camera_preflight import ensure_camera_device_available
from ugv_follower.utils.config_utils import get_project_root

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_CONFIG_PATH = get_project_root() / "configs" / "calibration_config.yaml"
_IMAGES_DIR = get_project_root() / "calibration" / "images"
_STREAM_PORT = 8080
_JPEG_QUALITY = 80
_CORNER_CRITERIA = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001,
)


# ---------------------------------------------------------------------------
# Shared state (written by capture thread, read by HTTP handlers)
# ---------------------------------------------------------------------------


class _CaptureState:
    """Thread-safe container for the latest frame and detection result."""

    def __init__(self, start_count: int = 0) -> None:
        self._lock = threading.Lock()
        self._raw_frame: cv2.typing.MatLike | None = None
        self._annotated_frame: cv2.typing.MatLike | None = None
        self._corners_visible: bool = False
        self._saved_count: int = start_count

    # -- writers (called from capture thread) --------------------------------

    def update_frame(
        self,
        raw: cv2.typing.MatLike,
        annotated: cv2.typing.MatLike,
        corners: bool,
    ) -> None:
        """Replace the stored frames and corner-detection result atomically.

        Parameters
        ----------
        raw : cv2.typing.MatLike
            Unmodified frame straight from the camera.
        annotated : cv2.typing.MatLike
            Copy of *raw* with checkerboard corners drawn (if detected).
        corners : bool
            ``True`` if corners were detected in this frame.
        """
        with self._lock:
            self._raw_frame = raw
            self._annotated_frame = annotated
            self._corners_visible = corners

    # -- readers / actions (called from HTTP handlers) -----------------------

    def status(self) -> dict[str, object]:
        """Return a snapshot of the current capture state.

        Returns
        -------
        dict
            ``{"count": int, "corners_visible": bool}``
        """
        with self._lock:
            return {
                "count": self._saved_count,
                "corners_visible": self._corners_visible,
            }

    def try_capture(self) -> dict[str, object]:
        """Save the current raw frame if corners are visible. Thread-safe."""
        # Minimize time holding the lock: copy frame and index, then write.
        with self._lock:
            if not self._corners_visible or self._raw_frame is None:
                return {"saved": False, "count": self._saved_count}
            idx = self._saved_count
            frame = self._raw_frame.copy()
            path = _IMAGES_DIR / f"frame_{idx:04d}.jpg"

        ok = cv2.imwrite(str(path), frame)
        if not ok:
            logger.error(f"Failed to save {path}")
            # Count remains unchanged on failure.
            with self._lock:
                return {"saved": False, "count": self._saved_count}

        with self._lock:
            self._saved_count += 1
            logger.info(f"Saved {path}  (total: {self._saved_count})")
            return {"saved": True, "count": self._saved_count}

    def get_annotated_jpeg(self) -> bytes | None:
        """JPEG-encode the latest annotated frame and return the raw bytes.

        Returns
        -------
        bytes or None
            JPEG bytes ready to send in an MJPEG stream, or ``None`` if no
            frame has been captured yet.
        """
        with self._lock:
            frame = self._annotated_frame
        if frame is None:
            return None
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, _JPEG_QUALITY])
        return bytes(buf) if ok else None


# ---------------------------------------------------------------------------
# Capture thread
# ---------------------------------------------------------------------------


def _run_capture(
    cap: cv2.VideoCapture,
    state: _CaptureState,
    board_size: tuple[int, int],
    stop_event: threading.Event,
) -> None:
    """Continuously grab frames, detect checkerboard corners, and update shared state.

    Intended to run in a daemon thread. Loops until *stop_event* is set, reading
    frames from *cap*, running ``cv2.findChessboardCorners`` on each, and calling
    ``state.update_frame`` so the HTTP handlers always have a fresh annotated frame
    and corner-detection result to serve.

    Parameters
    ----------
    cap : cv2.VideoCapture
        Opened camera to read frames from.
    state : _CaptureState
        Shared state object updated on every iteration.
    board_size : tuple[int, int]
        ``(cols, rows)`` of inner corners on the calibration checkerboard.
    stop_event : threading.Event
        Set this event to signal the loop to exit cleanly.
    """
    while not stop_event.is_set():
        ok, frame = cap.read()
        if not ok:
            logger.warning("VideoCapture.read() failed — retrying...")
            time.sleep(0.05)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray,
            board_size,
            None,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        annotated = frame.copy()
        if found:
            assert corners is not None
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), _CORNER_CRITERIA
            )
            cv2.drawChessboardCorners(annotated, board_size, corners, found)

        state.update_frame(frame, annotated, bool(found))


# ---------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------


def _make_handler(
    state: _CaptureState, stream_fps: int = 30
) -> type[BaseHTTPRequestHandler]:
    """Return a handler class closed over *state*."""

    class Handler(BaseHTTPRequestHandler):
        def log_message(
            self, format: str, *args: object
        ) -> None:  # silence access log  # noqa: A002
            pass

        def do_GET(self) -> None:  # noqa: N802
            if self.path == "/stream":
                self._serve_stream()
            elif self.path == "/capture":
                self._serve_json(state.try_capture())
            elif self.path in ("/status", "/"):
                self._serve_json(state.status())
            else:
                self.send_error(404)

        def _serve_json(self, data: dict[str, object]) -> None:
            body = json.dumps(data).encode()
            self.send_response(200)
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
                    time.sleep(1 / stream_fps)
            except (BrokenPipeError, ConnectionResetError):
                pass  # client disconnected

    return Handler


# ---------------------------------------------------------------------------
# Hardware helpers
# ---------------------------------------------------------------------------


def _zero_pan_tilt(port: str, baud_rate: int) -> None:
    """Connect to the UGV sub-controller, centre the pan-tilt, then disconnect.

    Parameters
    ----------
    port : str
        Serial port the UGV sub-controller is connected to (e.g. ``/dev/ttyAMA0``).
    baud_rate : int
        Baud rate for the serial connection.
    """
    logger.info(f"Zeroing pan-tilt via {port}...")
    controller = UGVController(port=port, baud_rate=baud_rate, chassis_module=2)
    controller.connect()
    controller.set_pan_tilt(0.0, 0.0)
    time.sleep(0.5)  # Allow servos to reach centre before we disconnect
    controller.disconnect()
    logger.info("Pan-tilt zeroed. Serial connection closed.")


def _open_camera(device_index: int, width: int, height: int) -> cv2.VideoCapture:
    """Open a VideoCapture at the requested resolution and return it.

    Parameters
    ----------
    device_index : int
        ``cv2.VideoCapture`` device index for the Waveshare RGB camera.
    width : int
        Requested frame width in pixels.
    height : int
        Requested frame height in pixels.

    Returns
    -------
    cv2.VideoCapture
        Opened and configured capture object.

    Raises
    ------
    SystemExit
        If the device cannot be opened.
    """
    logger.info(f"Opening camera (device_index={device_index})...")
    cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        logger.error(f"Could not open VideoCapture(index={device_index})")
        sys.exit(1)
    logger.info("Camera opened.")
    return cap


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    # -- Load config ---------------------------------------------------------
    if not _CONFIG_PATH.exists():
        logger.error(f"Config not found: {_CONFIG_PATH}")
        sys.exit(1)
    with _CONFIG_PATH.open() as f:
        cfg = yaml.safe_load(f)

    board_cols, board_rows = cfg["checkerboard"]["inner_corners"]
    board_size = (board_cols, board_rows)
    device_index = int(cfg["camera"]["device_index"])
    res_w, res_h = cfg["camera"]["resolution"]
    stream_fps = int(cfg["camera"].get("stream_fps", 30))
    ugv_port = str(cfg["ugv"]["port"])
    ugv_baud = int(cfg["ugv"]["baud_rate"])

    # -- Zero pan-tilt -------------------------------------------------------
    _zero_pan_tilt(ugv_port, ugv_baud)

    # -- Camera preflight ----------------------------------------------------
    try:
        ensure_camera_device_available(f"/dev/video{device_index}")
    except RuntimeError as exc:
        logger.error(str(exc))
        sys.exit(1)

    # -- Open camera ---------------------------------------------------------
    cap = _open_camera(device_index, res_w, res_h)

    # -- Prepare output dir --------------------------------------------------
    _IMAGES_DIR.mkdir(parents=True, exist_ok=True)
    existing = [int(p.stem.split("_")[1]) for p in _IMAGES_DIR.glob("frame_????.jpg")]
    start_count = max(existing) + 1 if existing else 0
    if existing:
        logger.info(
            f"Found {len(existing)} existing image(s) in {_IMAGES_DIR}, starting from frame_{start_count:04d}."
        )

    # -- Start capture thread ------------------------------------------------
    state = _CaptureState(start_count=start_count)
    stop_event = threading.Event()
    capture_thread = threading.Thread(
        target=_run_capture,
        args=(cap, state, board_size, stop_event),
        daemon=True,
    )
    capture_thread.start()
    logger.info("Capture thread started.")

    # -- Start HTTP server ---------------------------------------------------
    server = ThreadingHTTPServer(
        ("0.0.0.0", _STREAM_PORT), _make_handler(state, stream_fps)
    )
    logger.info(
        f"HTTP server running on port {_STREAM_PORT}. "
        f"Open http://<pi-ip>:{_STREAM_PORT}/stream in your browser."
    )
    logger.info(
        "Endpoints: /stream (MJPEG), "
        "/capture (save if corners visible), "
        "/status (JSON count + corners_visible)"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        stop_event.set()
        capture_thread.join(timeout=2)
        cap.release()
        server.server_close()
        logger.info(f"Done. {state.status()['count']} image(s) saved to {_IMAGES_DIR}.")


if __name__ == "__main__":
    main()
