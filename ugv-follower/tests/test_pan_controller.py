"""Tests for ugv_follower.control.pan_controller.

Tests cover:
- heading sign: centroid left/right of principal point
- heading near centre: ~zero
- tilt correction: zero tilt and non-zero tilt
- deadband: inside and outside
- clamp: at min and max limits
- PanController.update: no-detection hold, deadband hold, detection update, clamped output

Run with:
    cd ugv-follower
    pytest tests/test_pan_controller.py -v
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from ugv_follower.control.pan_controller import (
    PanController,
    apply_tilt_correction,
    clamp_pan,
    within_deadband,
)
from ugv_follower.utils.fisheye_utils import pixel_to_bearing_deg

# ---------------------------------------------------------------------------
# Shared test fixtures
# ---------------------------------------------------------------------------

# Synthetic pinhole-equivalent intrinsics: zero distortion so that
# pixel_to_bearing_deg results match plain atan((u - cx) / fx).
_FX = 500.0
_CX = 320.0
_CY = 240.0
_K = np.array([[_FX, 0.0, _CX], [0.0, _FX, _CY], [0.0, 0.0, 1.0]])
_D = np.zeros((4, 1), dtype=np.float64)

_CMD_MIN = -45.0
_CMD_MAX = 45.0
_DB_POS = 5.0
_DB_NEG = -5.0


def _make_controller(tilt_deg: float = 0.0) -> PanController:
    return PanController(
        K=_K,
        D=_D,
        cmd_min_deg=_CMD_MIN,
        cmd_max_deg=_CMD_MAX,
        dead_band_pos_deg=_DB_POS,
        dead_band_neg_deg=_DB_NEG,
        tilt_deg=tilt_deg,
    )


# ---------------------------------------------------------------------------
# pixel_to_bearing_deg — heading sign checks
# ---------------------------------------------------------------------------


def test_heading_sign_right_of_centre() -> None:
    """Centroid right of cx → positive heading (target right)."""
    heading = pixel_to_bearing_deg(_CX + 100.0, _CY, _K, _D)
    assert heading > 0.0


def test_heading_sign_left_of_centre() -> None:
    """Centroid left of cx → negative heading (target left)."""
    heading = pixel_to_bearing_deg(_CX - 100.0, _CY, _K, _D)
    assert heading < 0.0


def test_heading_at_centre_is_zero() -> None:
    """Centroid exactly at (cx, cy) → heading is 0°."""
    heading = pixel_to_bearing_deg(_CX, _CY, _K, _D)
    assert heading == pytest.approx(0.0, abs=1e-9)


def test_heading_known_value() -> None:
    """100 px right of cx with D=0 uses equidistant model: theta = r/f = 100/500 = 0.2 rad ≈ 11.46°.

    The fisheye model (even with D=zeros) differs from pinhole: cv2.fisheye.undistortPoints
    applies equidistant projection (theta = r/f), so the bearing equals theta, not atan(r/f).
    """
    expected = math.degrees(100.0 / _FX)  # equidistant: theta = r/f
    heading = pixel_to_bearing_deg(_CX + 100.0, _CY, _K, _D)
    assert heading == pytest.approx(expected, abs=1e-4)


# ---------------------------------------------------------------------------
# apply_tilt_correction
# ---------------------------------------------------------------------------


def test_tilt_correction_zero_tilt() -> None:
    """Zero tilt → heading passes through unchanged."""
    assert apply_tilt_correction(15.0, 0.0) == pytest.approx(15.0)


def test_tilt_correction_nonzero() -> None:
    """Non-zero tilt → heading * cos(tilt_rad)."""
    tilt = 20.0
    expected = 15.0 * math.cos(math.radians(tilt))
    assert apply_tilt_correction(15.0, tilt) == pytest.approx(expected)


# ---------------------------------------------------------------------------
# within_deadband
# ---------------------------------------------------------------------------


def test_within_deadband_inside_positive() -> None:
    """Heading within positive half of deadband → True."""
    assert within_deadband(3.0, _DB_POS, _DB_NEG) is True


def test_within_deadband_inside_negative() -> None:
    """Heading within negative half of deadband → True."""
    assert within_deadband(-3.0, _DB_POS, _DB_NEG) is True


def test_within_deadband_at_boundary() -> None:
    """Heading exactly at boundary → True (inclusive)."""
    assert within_deadband(_DB_POS, _DB_POS, _DB_NEG) is True
    assert within_deadband(_DB_NEG, _DB_POS, _DB_NEG) is True


def test_within_deadband_outside() -> None:
    """Heading beyond boundary → False."""
    assert within_deadband(6.0, _DB_POS, _DB_NEG) is False
    assert within_deadband(-6.0, _DB_POS, _DB_NEG) is False


# ---------------------------------------------------------------------------
# clamp_pan
# ---------------------------------------------------------------------------


def test_clamp_pan_at_max() -> None:
    """Pan above cmd_max → clamped to cmd_max."""
    assert clamp_pan(60.0, _CMD_MIN, _CMD_MAX) == pytest.approx(_CMD_MAX)


def test_clamp_pan_at_min() -> None:
    """Pan below cmd_min → clamped to cmd_min."""
    assert clamp_pan(-60.0, _CMD_MIN, _CMD_MAX) == pytest.approx(_CMD_MIN)


def test_clamp_pan_in_range() -> None:
    """Pan within limits → returned unchanged."""
    assert clamp_pan(20.0, _CMD_MIN, _CMD_MAX) == pytest.approx(20.0)


# ---------------------------------------------------------------------------
# PanController.update
# ---------------------------------------------------------------------------


class TestPanController:
    def test_no_detection_returns_none(self) -> None:
        """update(None, None) → None (hold, no servo command)."""
        ctrl = _make_controller()
        result = ctrl.update(None, None)
        assert result is None

    def test_no_detection_does_not_change_pan(self) -> None:
        """Pan position is unchanged after a no-detection call."""
        ctrl = _make_controller()
        initial = ctrl.current_pan_deg
        ctrl.update(None, None)
        assert ctrl.current_pan_deg == pytest.approx(initial)

    def test_within_deadband_returns_none(self) -> None:
        """Heading within ±5° deadband → None (hold)."""
        ctrl = _make_controller()
        # 2 px right with fx=500 → atan(2/500) ≈ 0.23° — well inside ±5° deadband
        result = ctrl.update(_CX + 2.0, _CY)
        assert result is None

    def test_detection_outside_deadband_returns_command(self) -> None:
        """Clear detection outside deadband → returns a float pan command."""
        ctrl = _make_controller()
        # 100 px right → ~11.3° heading, outside ±5° deadband
        result = ctrl.update(_CX + 100.0, _CY)
        assert result is not None
        assert isinstance(result, float)

    def test_detection_right_gives_positive_pan(self) -> None:
        """Target right of centre → positive pan command."""
        ctrl = _make_controller()
        result = ctrl.update(_CX + 100.0, _CY)
        assert result is not None
        assert result > 0.0

    def test_detection_left_gives_negative_pan(self) -> None:
        """Target left of centre → negative pan command."""
        ctrl = _make_controller()
        result = ctrl.update(_CX - 100.0, _CY)
        assert result is not None
        assert result < 0.0

    def test_extreme_heading_clamped_at_max(self) -> None:
        """Heading that would push pan beyond cmd_max → clamped at cmd_max."""
        ctrl = _make_controller()
        # Drive pan to near-max first, then push it further
        # 40° heading is outside deadband and near limit
        # atan(x/500) = 40° → x = 500 * tan(40°) ≈ 420 px
        extreme_u = _CX + 500.0 * math.tan(math.radians(40.0))
        # First call accumulates 40° into pan
        ctrl.update(extreme_u, _CY)
        # Second call would push well past 45° limit
        result = ctrl.update(extreme_u, _CY)
        assert result is not None
        assert result == pytest.approx(_CMD_MAX)

    def test_pan_position_tracked_across_updates(self) -> None:
        """current_pan_deg reflects the last issued command."""
        ctrl = _make_controller()
        # 100 px right → ~11.3° → pan moves from 0 to ~11.3°
        result = ctrl.update(_CX + 100.0, _CY)
        assert result is not None
        assert ctrl.current_pan_deg == pytest.approx(result)
