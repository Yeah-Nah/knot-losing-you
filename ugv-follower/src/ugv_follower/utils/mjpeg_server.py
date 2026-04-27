"""Thread-safe MJPEG streaming server for the ugv-follower pipeline."""

from __future__ import annotations

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import TYPE_CHECKING

import cv2
from loguru import logger

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

_JPEG_QUALITY = 80


class MjpegServer:
    """Serves a single MJPEG stream at ``GET /stream``.

    Frames are pushed by the pipeline thread via :meth:`push_frame`; the HTTP
    server thread reads the latest frame on each streaming tick.  The server
    runs in a daemon thread so it stops automatically when the main process
    exits.

    Parameters
    ----------
    port : int
        TCP port to listen on.
    """

    def __init__(self, port: int) -> None:
        self._port = port
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._server: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        """Start the HTTP server in a daemon thread."""
        self._server = ThreadingHTTPServer(
            ("0.0.0.0", self._port), self._make_handler()
        )
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        logger.info("MJPEG stream started on http://0.0.0.0:{}/stream", self._port)

    def stop(self) -> None:
        """Shut down the HTTP server. Safe to call even if never started."""
        if self._server is not None:
            self._server.shutdown()
            self._server = None
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None

    def push_frame(self, frame: NDArray[np.uint8]) -> None:
        """JPEG-encode *frame* and store it for the next stream response.

        Parameters
        ----------
        frame : NDArray[np.uint8]
            BGR image to encode and serve.
        """
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, _JPEG_QUALITY])
        if ok:
            with self._lock:
                self._jpeg = bytes(buf)

    def _get_jpeg(self) -> bytes | None:
        with self._lock:
            return self._jpeg

    def _make_handler(self) -> type[BaseHTTPRequestHandler]:
        server = self

        class _Handler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args: object) -> None:  # noqa: A002
                pass

            def do_GET(self) -> None:  # noqa: N802
                if self.path == "/stream":
                    self._serve_stream()
                else:
                    self.send_error(404)

            def _serve_stream(self) -> None:
                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=frame",
                )
                self.end_headers()
                try:
                    while True:
                        jpeg = server._get_jpeg()
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
                        time.sleep(0.033)  # ~30 fps cap
                except (BrokenPipeError, ConnectionResetError):
                    pass

        return _Handler
