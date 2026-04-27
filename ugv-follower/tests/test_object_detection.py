"""Tests for ugv_follower.inference.object_detection.select_target_centroid.

Module under test:
    ugv_follower.inference.object_detection.select_target_centroid

Test groups:
    - No detections: empty results list, or results with boxes=None/empty
    - Single detection: correct centroid computed from box coordinates
    - Multiple detections: highest-confidence box selected

Run with:
    cd ugv-follower
    pytest tests/test_object_detection.py -v

Notes:
    No model weights or hardware required. Results objects are mocked with
    numpy arrays so tests run offline.
"""

from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from ugv_follower.inference.object_detection import select_target_centroid

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_results(
    boxes: list[tuple[float, float, float, float, float]],
) -> list[Any]:
    """Return a mocked Ultralytics Results list.

    Parameters
    ----------
    boxes : list of (x1, y1, x2, y2, conf)
        One entry per detected box.
    """
    result = MagicMock()
    if not boxes:
        result.boxes = None
        return [result]
    mock_boxes = MagicMock()
    mock_boxes.xyxy = np.array([[b[0], b[1], b[2], b[3]] for b in boxes], dtype=float)
    mock_boxes.conf = np.array([b[4] for b in boxes], dtype=float)
    mock_boxes.__len__ = lambda _: len(boxes)
    result.boxes = mock_boxes
    return [result]


# ---------------------------------------------------------------------------
# Tests: no detections
# ---------------------------------------------------------------------------


def test_empty_results_returns_none_pair() -> None:
    """Empty results list yields (None, None)."""
    assert select_target_centroid([]) == (None, None)


def test_boxes_none_returns_none_pair() -> None:
    """Results present but boxes=None yields (None, None)."""
    result = MagicMock()
    result.boxes = None
    assert select_target_centroid([result]) == (None, None)


def test_empty_boxes_returns_none_pair() -> None:
    """Results with zero boxes yields (None, None)."""
    assert select_target_centroid(_make_results([])) == (None, None)


# ---------------------------------------------------------------------------
# Tests: detection present
# ---------------------------------------------------------------------------


def test_single_detection_centroid() -> None:
    """Single box: centroid is the midpoint of (x1, y1, x2, y2)."""
    results = _make_results([(100.0, 50.0, 200.0, 150.0, 0.9)])
    u, v = select_target_centroid(results)
    assert u == pytest.approx(150.0)
    assert v == pytest.approx(100.0)


def test_picks_highest_confidence_detection() -> None:
    """Two boxes: the one with higher confidence has its centroid returned."""
    # First box: conf=0.6, centroid=(150, 100)
    # Second box: conf=0.9, centroid=(350, 200)
    results = _make_results(
        [
            (100.0, 50.0, 200.0, 150.0, 0.6),
            (300.0, 150.0, 400.0, 250.0, 0.9),
        ]
    )
    u, v = select_target_centroid(results)
    assert u == pytest.approx(350.0)
    assert v == pytest.approx(200.0)
