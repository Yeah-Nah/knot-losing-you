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
Run from the ugv-follower directory::

    python capture_calibration_images.py

The script reads calibration_config.yaml (same directory as this file) for the
board dimensions, camera device index, and UGV serial port. It zeros the
pan-tilt servo before opening the camera, then starts the HTTP server.

Press Ctrl-C to stop.
"""

from __future__ import annotations

import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import yaml
from loguru import logger

# Allow imports from src/ when running as a script from ugv-follower/
sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))
from control.ugv_controller import UGVController  # noqa: E402

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_SCRIPT_DIR = Path(__file__).resolve().parent
_CONFIG_PATH = _SCRIPT_DIR / "configs" / "calibration_config.yaml"
_IMAGES_DIR = _SCRIPT_DIR / "calibration" / "images"
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
        with self._lock:
            self._raw_frame = raw
            self._annotated_frame = annotated
            self._corners_visible = corners

    # -- readers / actions (called from HTTP handlers) -----------------------

    def status(self) -> dict[str, object]:
        with self._lock:
            return {
                "count": self._saved_count,
                "corners_visible": self._corners_visible,
            }

    def try_capture(self) -> dict[str, object]:
        """Save the current raw frame if corners are visible. Thread-safe."""
        with self._lock:
            if not self._corners_visible or self._raw_frame is None:
                return {"saved": False, "count": self._saved_count}
            idx = self._saved_count
            path = _IMAGES_DIR / f"frame_{idx:04d}.jpg"
            cv2.imwrite(str(path), self._raw_frame)
            self._saved_count += 1
            logger.info(f"Saved {path}  (total: {self._saved_count})")
            return {"saved": True, "count": self._saved_count}

    def get_annotated_jpeg(self) -> bytes | None:
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
    """Continuously grab frames, detect corners, and update shared state."""
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
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        annotated = frame.copy()
        if found:
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), _CORNER_CRITERIA
            )
            cv2.drawChessboardCorners(annotated, board_size, corners, found)

        state.update_frame(frame, annotated, bool(found))


# ---------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------


def _make_handler(state: _CaptureState, stream_fps: int = 30) -> type[BaseHTTPRequestHandler]:
    """Return a handler class closed over *state*."""

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: object) -> None:  # silence access log
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
    logger.info(f"Zeroing pan-tilt via {ugv_port}...")
    controller = UGVController(port=ugv_port, baud_rate=ugv_baud, chassis_module=2)
    controller.connect()
    controller.set_pan_tilt(0.0, 0.0)
    time.sleep(0.5)  # Allow servos to reach centre before we disconnect
    controller.disconnect()
    logger.info("Pan-tilt zeroed. Serial connection closed.")

    # -- Open camera ---------------------------------------------------------
    logger.info(f"Opening camera (device_index={device_index})...")
    cap = cv2.VideoCapture(device_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, res_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res_h)
    if not cap.isOpened():
        logger.error(f"Could not open VideoCapture(index={device_index})")
        sys.exit(1)
    logger.info("Camera opened.")

    # -- Prepare output dir --------------------------------------------------
    _IMAGES_DIR.mkdir(parents=True, exist_ok=True)
    existing = [int(p.stem.split("_")[1]) for p in _IMAGES_DIR.glob("frame_????.jpg")]
    start_count = max(existing) + 1 if existing else 0
    if existing:
        logger.info(f"Found {len(existing)} existing image(s) in {_IMAGES_DIR}, starting from frame_{start_count:04d}.")

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
    server = ThreadingHTTPServer(("0.0.0.0", _STREAM_PORT), _make_handler(state, stream_fps))
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
