"""Unit tests for WaveshareCamera.

Module under test
-----------------
ugv_follower.perception.waveshare_camera

Test groups
-----------
1  Before start    — get_frame returns None before start() is called.
2  start()         — opens VideoCapture with correct device index and properties;
                     cleans up on open failure; rejects a second start().
3  _capture_once() — publishes timestamped CameraFrame snapshots; keeps the
                     previous frame on read failure (no thread).
4  get_frame()     — grabber thread delivers the newest frame; None while reads fail.
5  stop()          — joins the grabber thread and releases the capture device;
                     keeps ownership of a grabber that outlives the join timeout.

Running
-------
All tests in this file (from ``ugv-follower/``)::

    pytest tests/test_waveshare_camera.py

Notes
-----
No hardware required.  cv2.VideoCapture and camera_preflight are mocked.
"""

from __future__ import annotations

import threading
import time
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from ugv_follower.perception.waveshare_camera import CameraFrame, WaveshareCamera

_DEVICE_INDEX = 0
_WIDTH = 1920
_HEIGHT = 1080
_FPS = 30
_MONOTONIC = "ugv_follower.perception.waveshare_camera.time.monotonic"


def _make_camera() -> WaveshareCamera:
    return WaveshareCamera(
        device_index=_DEVICE_INDEX,
        width=_WIDTH,
        height=_HEIGHT,
        fps=_FPS,
    )


def _make_mock_cap(ok: bool) -> MagicMock:
    """Return an opened mock capture whose read() succeeds or fails."""
    mock_cap = MagicMock()
    mock_cap.isOpened.return_value = True

    def _read() -> tuple[bool, np.ndarray | None]:
        time.sleep(0.001)  # simulate frame interval; avoid a GIL-hogging spin
        if not ok:
            return False, None
        return True, np.zeros((_HEIGHT, _WIDTH, 3), dtype=np.uint8)

    mock_cap.read.side_effect = _read
    return mock_cap


def _wait_for_frame(cam: WaveshareCamera, timeout_s: float = 1.0) -> CameraFrame:
    """Poll get_frame() until the grabber publishes a frame."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame = cam.get_frame()
        if frame is not None:
            return frame
        time.sleep(0.005)
    raise AssertionError(f"No frame published within {timeout_s}s")


# ---------------------------------------------------------------------------
# 1. Before start
# ---------------------------------------------------------------------------


def test_get_frame_before_start_returns_none() -> None:
    """get_frame() returns None when start() has not been called."""
    cam = _make_camera()
    assert cam.get_frame() is None


# ---------------------------------------------------------------------------
# 2. start()
# ---------------------------------------------------------------------------


@patch("ugv_follower.perception.waveshare_camera.ensure_camera_device_available")
@patch("ugv_follower.perception.waveshare_camera.cv2.VideoCapture")
def test_start_opens_capture_with_correct_device(
    mock_capture_cls: MagicMock,
    mock_preflight: MagicMock,
) -> None:
    """start() opens VideoCapture with the configured device index."""
    mock_cap = _make_mock_cap(ok=True)
    mock_capture_cls.return_value = mock_cap

    cam = _make_camera()
    cam.start()
    cam.stop()

    mock_capture_cls.assert_called_once()
    args = mock_capture_cls.call_args[0]
    assert args[0] == _DEVICE_INDEX


@patch("ugv_follower.perception.waveshare_camera.ensure_camera_device_available")
@patch("ugv_follower.perception.waveshare_camera.cv2.VideoCapture")
def test_start_sets_width_height_fps(
    mock_capture_cls: MagicMock,
    mock_preflight: MagicMock,
) -> None:
    """start() sets width, height, and fps on the VideoCapture object."""
    import cv2

    mock_cap = _make_mock_cap(ok=True)
    mock_capture_cls.return_value = mock_cap

    cam = _make_camera()
    cam.start()
    cam.stop()

    set_calls = mock_cap.set.call_args_list
    prop_ids = {c[0][0] for c in set_calls}
    assert cv2.CAP_PROP_FRAME_WIDTH in prop_ids
    assert cv2.CAP_PROP_FRAME_HEIGHT in prop_ids
    assert cv2.CAP_PROP_FPS in prop_ids


@patch("ugv_follower.perception.waveshare_camera.ensure_camera_device_available")
@patch("ugv_follower.perception.waveshare_camera.cv2.VideoCapture")
def test_start_runs_preflight(
    mock_capture_cls: MagicMock,
    mock_preflight: MagicMock,
) -> None:
    """start() calls ensure_camera_device_available with the correct device path."""
    mock_cap = _make_mock_cap(ok=True)
    mock_capture_cls.return_value = mock_cap

    cam = _make_camera()
    cam.start()
    cam.stop()

    mock_preflight.assert_called_once_with(f"/dev/video{_DEVICE_INDEX}")


@patch("ugv_follower.perception.waveshare_camera.ensure_camera_device_available")
@patch("ugv_follower.perception.waveshare_camera.cv2.VideoCapture")
def test_start_raises_when_capture_not_opened(
    mock_capture_cls: MagicMock,
    mock_preflight: MagicMock,
) -> None:
    """start() raises, releases the capture, and starts no thread if not opened."""
    mock_cap = MagicMock()
    mock_cap.isOpened.return_value = False
    mock_capture_cls.return_value = mock_cap

    cam = _make_camera()
    with pytest.raises(RuntimeError, match="Could not open Waveshare camera"):
        cam.start()

    mock_cap.release.assert_called_once()
    assert cam._thread is None


@patch("ugv_follower.perception.waveshare_camera.ensure_camera_device_available")
@patch("ugv_follower.perception.waveshare_camera.cv2.VideoCapture")
def test_start_twice_raises(
    mock_capture_cls: MagicMock,
    mock_preflight: MagicMock,
) -> None:
    """start() raises RuntimeError when the grabber thread is already running."""
    mock_capture_cls.return_value = _make_mock_cap(ok=True)

    cam = _make_camera()
    cam.start()
    try:
        with pytest.raises(RuntimeError, match="already running"):
            cam.start()
    finally:
        cam.stop()


# ---------------------------------------------------------------------------
# 3. _capture_once() — snapshot publication (no thread)
# ---------------------------------------------------------------------------


def test_capture_once_publishes_frame_with_timestamp() -> None:
    """A successful read publishes a CameraFrame stamped after read() returns."""
    cam = _make_camera()
    cam._cap = _make_mock_cap(ok=True)

    with patch(_MONOTONIC, return_value=123.0):
        assert cam._capture_once() is True

    frame = cam.get_frame()
    assert frame is not None
    assert frame.seq == 1
    assert frame.capture_time_monotonic == 123.0
    assert frame.image.shape == (_HEIGHT, _WIDTH, 3)


def test_capture_once_replaces_previous_frame() -> None:
    """Each successful read replaces the latest frame and advances seq."""
    cam = _make_camera()
    cam._cap = _make_mock_cap(ok=True)

    cam._capture_once()
    first = cam.get_frame()
    cam._capture_once()
    second = cam.get_frame()

    assert second is not None
    assert second.seq == 2
    assert second is not first


def test_capture_once_failure_keeps_previous_frame() -> None:
    """A failed read returns False and leaves the previous frame in place."""
    cam = _make_camera()
    mock_cap = _make_mock_cap(ok=True)
    cam._cap = mock_cap
    cam._capture_once()
    previous = cam.get_frame()

    mock_cap.read.side_effect = None
    mock_cap.read.return_value = (False, None)
    assert cam._capture_once() is False
    assert cam.get_frame() is previous


def test_camera_frame_age_uses_monotonic_clock() -> None:
    """CameraFrame.age_s is the time elapsed since capture."""
    frame = CameraFrame(
        image=np.zeros((1, 1, 3), dtype=np.uint8),
        capture_time_monotonic=10.0,
        seq=1,
    )
    with patch(_MONOTONIC, return_value=10.25):
        assert frame.age_s == pytest.approx(0.25)


# ---------------------------------------------------------------------------
# 4. get_frame() — grabber thread
# ---------------------------------------------------------------------------


@patch("ugv_follower.perception.waveshare_camera.ensure_camera_device_available")
@patch("ugv_follower.perception.waveshare_camera.cv2.VideoCapture")
def test_get_frame_returns_frame_on_success(
    mock_capture_cls: MagicMock,
    mock_preflight: MagicMock,
) -> None:
    """get_frame() returns the grabbed frame once the thread has read one."""
    mock_capture_cls.return_value = _make_mock_cap(ok=True)

    cam = _make_camera()
    cam.start()
    try:
        frame = _wait_for_frame(cam)
    finally:
        cam.stop()

    assert frame.image.shape == (_HEIGHT, _WIDTH, 3)


@patch("ugv_follower.perception.waveshare_camera.ensure_camera_device_available")
@patch("ugv_follower.perception.waveshare_camera.cv2.VideoCapture")
def test_get_frame_returns_none_on_read_failure(
    mock_capture_cls: MagicMock,
    mock_preflight: MagicMock,
) -> None:
    """get_frame() stays None while every VideoCapture.read() fails."""
    mock_capture_cls.return_value = _make_mock_cap(ok=False)

    cam = _make_camera()
    cam.start()
    try:
        time.sleep(0.05)
        assert cam.get_frame() is None
    finally:
        cam.stop()


@patch("ugv_follower.perception.waveshare_camera.ensure_camera_device_available")
@patch("ugv_follower.perception.waveshare_camera.cv2.VideoCapture")
def test_get_frame_returns_newest_frame(
    mock_capture_cls: MagicMock,
    mock_preflight: MagicMock,
) -> None:
    """The grabber keeps reading, so later calls return a newer frame."""
    mock_capture_cls.return_value = _make_mock_cap(ok=True)

    cam = _make_camera()
    cam.start()
    try:
        first = _wait_for_frame(cam)
        deadline = time.monotonic() + 1.0
        latest = first
        while latest.seq == first.seq and time.monotonic() < deadline:
            time.sleep(0.005)
            latest = cam.get_frame() or first
    finally:
        cam.stop()

    assert latest.seq > first.seq


# ---------------------------------------------------------------------------
# 5. stop()
# ---------------------------------------------------------------------------


@patch("ugv_follower.perception.waveshare_camera.ensure_camera_device_available")
@patch("ugv_follower.perception.waveshare_camera.cv2.VideoCapture")
def test_stop_releases_capture(
    mock_capture_cls: MagicMock,
    mock_preflight: MagicMock,
) -> None:
    """stop() joins the grabber, releases the capture, and clears the frame."""
    mock_cap = _make_mock_cap(ok=True)
    mock_capture_cls.return_value = mock_cap

    cam = _make_camera()
    cam.start()
    thread = cam._thread
    cam.stop()

    assert thread is not None
    assert not thread.is_alive()
    mock_cap.release.assert_called_once()
    assert cam.get_frame() is None  # frame cleared — get_frame returns None


def test_stop_before_start_is_safe() -> None:
    """stop() without a prior start() does not raise."""
    cam = _make_camera()
    cam.stop()  # must not raise


@patch("ugv_follower.perception.waveshare_camera._THREAD_JOIN_TIMEOUT_S", 0.05)
@patch("ugv_follower.perception.waveshare_camera.ensure_camera_device_available")
@patch("ugv_follower.perception.waveshare_camera.cv2.VideoCapture")
def test_stop_timeout_keeps_grabber_until_it_exits(
    mock_capture_cls: MagicMock,
    mock_preflight: MagicMock,
) -> None:
    """A grabber blocked in read() past the join timeout keeps ownership:
    no release, restart rejected, no late publish; a later stop() releases."""
    read_entered = threading.Event()
    unblock = threading.Event()
    mock_cap = MagicMock()
    mock_cap.isOpened.return_value = True

    def _blocked_read() -> tuple[bool, np.ndarray]:
        read_entered.set()
        unblock.wait()
        return True, np.zeros((_HEIGHT, _WIDTH, 3), dtype=np.uint8)

    mock_cap.read.side_effect = _blocked_read
    mock_capture_cls.return_value = mock_cap

    cam = _make_camera()
    cam.start()
    assert read_entered.wait(1.0)
    thread = cam._thread
    try:
        cam.stop()  # times out with read() still blocked

        mock_cap.release.assert_not_called()
        assert cam._thread is thread
        with pytest.raises(RuntimeError, match="already running"):
            cam.start()
    finally:
        unblock.set()
    assert thread is not None
    thread.join(1.0)

    assert not thread.is_alive()
    assert cam.get_frame() is None  # late read was not published
    cam.stop()
    mock_cap.release.assert_called_once()
    assert cam._thread is None
