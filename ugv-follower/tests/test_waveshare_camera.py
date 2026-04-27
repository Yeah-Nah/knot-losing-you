"""Unit tests for WaveshareCamera.

Module under test
-----------------
ugv_follower.perception.waveshare_camera

Test groups
-----------
1  Before start  — get_frame returns None before start() is called.
2  start()       — opens VideoCapture with correct device index and properties.
3  get_frame()   — returns frame on success; returns None when read fails.
4  stop()        — releases the capture device.

Running
-------
All tests in this file (from ``ugv-follower/``)::

    pytest tests/test_waveshare_camera.py

Notes
-----
No hardware required.  cv2.VideoCapture and camera_preflight are mocked.
"""

from __future__ import annotations

from unittest.mock import MagicMock, call, patch

import numpy as np
import pytest

from ugv_follower.perception.waveshare_camera import WaveshareCamera

_DEVICE_INDEX = 0
_WIDTH = 1920
_HEIGHT = 1080
_FPS = 30


def _make_camera() -> WaveshareCamera:
    return WaveshareCamera(
        device_index=_DEVICE_INDEX,
        width=_WIDTH,
        height=_HEIGHT,
        fps=_FPS,
    )


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
    mock_cap = MagicMock()
    mock_cap.isOpened.return_value = True
    mock_capture_cls.return_value = mock_cap

    cam = _make_camera()
    cam.start()

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

    mock_cap = MagicMock()
    mock_cap.isOpened.return_value = True
    mock_capture_cls.return_value = mock_cap

    cam = _make_camera()
    cam.start()

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
    mock_cap = MagicMock()
    mock_cap.isOpened.return_value = True
    mock_capture_cls.return_value = mock_cap

    cam = _make_camera()
    cam.start()

    mock_preflight.assert_called_once_with(f"/dev/video{_DEVICE_INDEX}")


@patch("ugv_follower.perception.waveshare_camera.ensure_camera_device_available")
@patch("ugv_follower.perception.waveshare_camera.cv2.VideoCapture")
def test_start_raises_when_capture_not_opened(
    mock_capture_cls: MagicMock,
    mock_preflight: MagicMock,
) -> None:
    """start() raises RuntimeError when VideoCapture.isOpened() returns False."""
    mock_cap = MagicMock()
    mock_cap.isOpened.return_value = False
    mock_capture_cls.return_value = mock_cap

    cam = _make_camera()
    with pytest.raises(RuntimeError, match="Could not open Waveshare camera"):
        cam.start()


# ---------------------------------------------------------------------------
# 3. get_frame()
# ---------------------------------------------------------------------------


@patch("ugv_follower.perception.waveshare_camera.ensure_camera_device_available")
@patch("ugv_follower.perception.waveshare_camera.cv2.VideoCapture")
def test_get_frame_returns_frame_on_success(
    mock_capture_cls: MagicMock,
    mock_preflight: MagicMock,
) -> None:
    """get_frame() returns the frame array when read() succeeds."""
    fake_frame = np.zeros((_HEIGHT, _WIDTH, 3), dtype=np.uint8)
    mock_cap = MagicMock()
    mock_cap.isOpened.return_value = True
    mock_cap.read.return_value = (True, fake_frame)
    mock_capture_cls.return_value = mock_cap

    cam = _make_camera()
    cam.start()
    frame = cam.get_frame()

    assert frame is not None
    assert frame.shape == (_HEIGHT, _WIDTH, 3)


@patch("ugv_follower.perception.waveshare_camera.ensure_camera_device_available")
@patch("ugv_follower.perception.waveshare_camera.cv2.VideoCapture")
def test_get_frame_returns_none_on_read_failure(
    mock_capture_cls: MagicMock,
    mock_preflight: MagicMock,
) -> None:
    """get_frame() returns None when VideoCapture.read() signals failure."""
    mock_cap = MagicMock()
    mock_cap.isOpened.return_value = True
    mock_cap.read.return_value = (False, None)
    mock_capture_cls.return_value = mock_cap

    cam = _make_camera()
    cam.start()
    assert cam.get_frame() is None


# ---------------------------------------------------------------------------
# 4. stop()
# ---------------------------------------------------------------------------


@patch("ugv_follower.perception.waveshare_camera.ensure_camera_device_available")
@patch("ugv_follower.perception.waveshare_camera.cv2.VideoCapture")
def test_stop_releases_capture(
    mock_capture_cls: MagicMock,
    mock_preflight: MagicMock,
) -> None:
    """stop() calls release() on the VideoCapture and clears the reference."""
    mock_cap = MagicMock()
    mock_cap.isOpened.return_value = True
    mock_capture_cls.return_value = mock_cap

    cam = _make_camera()
    cam.start()
    cam.stop()

    mock_cap.release.assert_called_once()
    assert cam.get_frame() is None  # cap cleared — get_frame returns None


def test_stop_before_start_is_safe() -> None:
    """stop() without a prior start() does not raise."""
    cam = _make_camera()
    cam.stop()  # must not raise
