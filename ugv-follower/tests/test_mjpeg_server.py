"""Unit tests for MjpegServer.

Module under test
-----------------
ugv_follower.utils.mjpeg_server — Thread-safe MJPEG HTTP streaming server.

Test groups
-----------
1  push_frame  — encodes a frame and stores it; no-op before any frame pushed.
2  lifecycle   — start/stop does not raise; double-stop is safe.
3  stream      — GET /stream returns 200 multipart; 404 for unknown paths.

Running
-------
All tests in this file (from ``ugv-follower/``)::

    pytest tests/test_mjpeg_server.py

Notes
-----
No hardware required.  The HTTP server binds to an ephemeral port (0) so
tests never conflict with real deployments.
"""

from __future__ import annotations

import socket
import time

import numpy as np
import pytest

from ugv_follower.utils.mjpeg_server import MjpegServer


def _free_port() -> int:
    """Return an OS-assigned free TCP port."""
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _blank_frame(h: int = 64, w: int = 64) -> np.ndarray:
    return np.zeros((h, w, 3), dtype=np.uint8)


# ---------------------------------------------------------------------------
# 1. push_frame
# ---------------------------------------------------------------------------


def test_push_frame_stores_jpeg() -> None:
    server = MjpegServer(port=_free_port())
    assert server._get_jpeg() is None

    server.push_frame(_blank_frame())

    jpeg = server._get_jpeg()
    assert jpeg is not None
    assert jpeg[:2] == b"\xff\xd8"  # JPEG magic bytes


def test_push_frame_replaces_previous() -> None:
    server = MjpegServer(port=_free_port())
    frame_a = _blank_frame()
    frame_b = np.full((64, 64, 3), 128, dtype=np.uint8)

    server.push_frame(frame_a)
    first = server._get_jpeg()
    server.push_frame(frame_b)
    second = server._get_jpeg()

    assert first != second


# ---------------------------------------------------------------------------
# 2. lifecycle
# ---------------------------------------------------------------------------


def test_start_stop_clean() -> None:
    port = _free_port()
    server = MjpegServer(port=port)
    server.start()
    time.sleep(0.05)  # let daemon thread bind
    server.stop()  # must not raise


def test_double_stop_is_safe() -> None:
    server = MjpegServer(port=_free_port())
    server.stop()  # never started — must not raise
    server.stop()  # idempotent


# ---------------------------------------------------------------------------
# 3. HTTP responses
# ---------------------------------------------------------------------------


def _get(port: int, path: str, read_bytes: int = 512, timeout: float = 2.0) -> tuple[int, bytes]:
    """Make a raw GET request and return (status_code, body_prefix)."""
    with socket.create_connection(("127.0.0.1", port), timeout=timeout) as sock:
        sock.sendall(f"GET {path} HTTP/1.0\r\nHost: localhost\r\n\r\n".encode())
        data = b""
        deadline = time.monotonic() + timeout
        while len(data) < read_bytes and time.monotonic() < deadline:
            chunk = sock.recv(read_bytes - len(data))
            if not chunk:
                break
            data += chunk
    first_line = data.split(b"\r\n", 1)[0].decode()
    code = int(first_line.split()[1])
    return code, data


def test_stream_endpoint_returns_200() -> None:
    port = _free_port()
    server = MjpegServer(port=port)
    server.push_frame(_blank_frame())
    server.start()
    time.sleep(0.05)
    try:
        code, body = _get(port, "/stream")
        assert code == 200
        assert b"multipart/x-mixed-replace" in body
    finally:
        server.stop()


def test_unknown_path_returns_404() -> None:
    port = _free_port()
    server = MjpegServer(port=port)
    server.start()
    time.sleep(0.05)
    try:
        code, _ = _get(port, "/not-a-real-path")
        assert code == 404
    finally:
        server.stop()
