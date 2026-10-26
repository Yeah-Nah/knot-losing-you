"""Unit tests for Pipeline camera-frame freshness.

Module under test
-----------------
ugv_follower.pipeline (``Pipeline._read_latest_frame``)

Running
-------
From ``ugv-follower/``::

    pytest tests/test_pipeline_frame_age.py

Notes
-----
No hardware required.  The Pipeline is built with ``object.__new__`` and a
WaveshareCamera whose capture is mocked.
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import numpy as np

from ugv_follower.perception.waveshare_camera import WaveshareCamera
from ugv_follower.pipeline import _MAX_FRAME_AGE_S, Pipeline

_MONOTONIC = "ugv_follower.perception.waveshare_camera.time.monotonic"


def _make_pipeline(camera: WaveshareCamera) -> Pipeline:
    pipeline = object.__new__(Pipeline)  # skip __init__, no hardware needed
    pipeline._camera = camera
    pipeline._frame_stale = False
    return pipeline


def test_read_latest_frame_drops_frame_after_prolonged_capture_failure() -> None:
    """A good capture followed by failing reads stops yielding the old image
    once it exceeds the max age, so the no-detection path holds pan."""
    camera = WaveshareCamera(device_index=0, width=4, height=4, fps=30)
    mock_cap = MagicMock()
    mock_cap.read.return_value = (True, np.zeros((4, 4, 3), dtype=np.uint8))
    camera._cap = mock_cap
    pipeline = _make_pipeline(camera)

    with patch(_MONOTONIC, return_value=100.0):
        assert camera._capture_once() is True
        assert pipeline._read_latest_frame() is not None

    mock_cap.read.return_value = (False, None)
    camera._stop_event.wait = MagicMock()  # skip the failure backoff
    with patch(_MONOTONIC, return_value=100.0 + _MAX_FRAME_AGE_S + 0.1):
        assert camera._capture_once() is False
        assert pipeline._read_latest_frame() is None
        assert pipeline._frame_stale is True

    mock_cap.read.return_value = (True, np.zeros((4, 4, 3), dtype=np.uint8))
    with patch(_MONOTONIC, return_value=200.0):
        assert camera._capture_once() is True
        assert pipeline._read_latest_frame() is not None
        assert pipeline._frame_stale is False
