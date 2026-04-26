"""Unit tests for LiDAR coordinate-system conversion helpers.

Module under test
-----------------
ugv_follower.perception.lidar_geometry — Converts raw D500 polar coordinates
into rover body-frame Cartesian coordinates and filters the forward obstacle arc.

Test groups
-----------
1  TestWrap360               — maps any float to [0, 360).
2  TestWrap180               — maps any float to (-180, 180].
3  TestLidarPointToBodyFrame — polar → Cartesian, mounting offset, mm→m conversion.
4  TestFilterForwardArc      — arc inclusion/exclusion and wrap-boundary edge cases.
5  TestNearestForwardPoint   — distance filtering and nearest-point selection.

Running
-------
All tests in this file (from ``ugv-follower/``)::

    pytest tests/test_lidar_geometry.py

Verbose with short tracebacks::

    pytest tests/test_lidar_geometry.py -v --tb=short

Run a specific group::

    pytest tests/test_lidar_geometry.py::TestWrap360
    pytest tests/test_lidar_geometry.py::TestFilterForwardArc

Notes
-----
No physical hardware required.  All tests are pure-function unit tests and run
offline without any serial devices attached.
"""

from __future__ import annotations

import math

import pytest

from ugv_follower.perception.lidar_geometry import (
    BodyFramePoint,
    filter_forward_arc,
    lidar_point_to_body_frame,
    nearest_forward_point,
    wrap_180,
    wrap_360,
)


def _pt(angle: float, distance: int = 1000, intensity: int = 100) -> dict:
    """Convenience factory for a raw LidarPoint dict."""
    return {"angle": angle, "distance": distance, "intensity": intensity}


def _body(bearing_deg: float, distance_m: float = 1.0) -> BodyFramePoint:
    """Convenience factory for a BodyFramePoint (x_m/y_m not checked in arc tests)."""
    theta_rad = math.radians(bearing_deg)
    x_m = distance_m * math.cos(theta_rad)
    y_m = -distance_m * math.sin(theta_rad)
    return {
        "x_m": x_m,
        "y_m": y_m,
        "distance_m": distance_m,
        "bearing_deg": bearing_deg,
    }


# ---------------------------------------------------------------------------
# Group 1 — wrap_360
# ---------------------------------------------------------------------------


class TestWrap360:
    def test_zero_unchanged(self) -> None:
        assert wrap_360(0.0) == pytest.approx(0.0)

    def test_360_wraps_to_zero(self) -> None:
        assert wrap_360(360.0) == pytest.approx(0.0)

    def test_720_wraps_to_zero(self) -> None:
        assert wrap_360(720.0) == pytest.approx(0.0)

    def test_negative_90_becomes_270(self) -> None:
        assert wrap_360(-90.0) == pytest.approx(270.0)

    def test_negative_180_becomes_180(self) -> None:
        assert wrap_360(-180.0) == pytest.approx(180.0)

    def test_fractional_passthrough(self) -> None:
        assert wrap_360(45.5) == pytest.approx(45.5)

    def test_just_below_360_unchanged(self) -> None:
        assert wrap_360(359.9) == pytest.approx(359.9)


# ---------------------------------------------------------------------------
# Group 2 — wrap_180
# ---------------------------------------------------------------------------


class TestWrap180:
    def test_zero_unchanged(self) -> None:
        assert wrap_180(0.0) == pytest.approx(0.0)

    def test_180_stays_positive(self) -> None:
        """180° is included in (-180, 180] as +180, not -180."""
        assert wrap_180(180.0) == pytest.approx(180.0)

    def test_181_becomes_negative(self) -> None:
        assert wrap_180(181.0) == pytest.approx(-179.0)

    def test_270_becomes_minus_90(self) -> None:
        assert wrap_180(270.0) == pytest.approx(-90.0)

    def test_360_wraps_to_zero(self) -> None:
        assert wrap_180(360.0) == pytest.approx(0.0)

    def test_negative_90_unchanged(self) -> None:
        assert wrap_180(-90.0) == pytest.approx(-90.0)

    def test_negative_180_becomes_plus_180(self) -> None:
        """−180° should map to +180° (the included boundary)."""
        assert wrap_180(-180.0) == pytest.approx(180.0)

    def test_arbitrary_large_positive(self) -> None:
        assert wrap_180(540.0) == pytest.approx(180.0)


# ---------------------------------------------------------------------------
# Group 3 — lidar_point_to_body_frame
# ---------------------------------------------------------------------------


class TestLidarPointToBodyFrame:
    """Cardinal-angle correctness and mounting-offset application."""

    def test_zero_deg_no_offset_points_forward(self) -> None:
        """0° with no offset → pure forward (+x), zero lateral."""
        bp = lidar_point_to_body_frame(_pt(0.0, distance=1000), mounting_offset_deg=0.0)
        assert bp["x_m"] == pytest.approx(1.0, abs=1e-9)
        assert bp["y_m"] == pytest.approx(0.0, abs=1e-9)
        assert bp["bearing_deg"] == pytest.approx(0.0)

    def test_90_deg_no_offset_points_right(self) -> None:
        """90° CW with no offset → to the right (+bearing, −y)."""
        bp = lidar_point_to_body_frame(
            _pt(90.0, distance=1000), mounting_offset_deg=0.0
        )
        assert bp["x_m"] == pytest.approx(0.0, abs=1e-9)
        assert bp["y_m"] == pytest.approx(-1.0, abs=1e-9)
        assert bp["bearing_deg"] == pytest.approx(90.0)

    def test_180_deg_no_offset_points_rearward(self) -> None:
        """180° with no offset → behind (−x)."""
        bp = lidar_point_to_body_frame(
            _pt(180.0, distance=1000), mounting_offset_deg=0.0
        )
        assert bp["x_m"] == pytest.approx(-1.0, abs=1e-9)
        assert bp["y_m"] == pytest.approx(0.0, abs=1e-9)
        assert bp["bearing_deg"] == pytest.approx(180.0)

    def test_270_deg_no_offset_points_left(self) -> None:
        """270° with no offset → to the left (−bearing, +y)."""
        bp = lidar_point_to_body_frame(
            _pt(270.0, distance=1000), mounting_offset_deg=0.0
        )
        assert bp["x_m"] == pytest.approx(0.0, abs=1e-9)
        assert bp["y_m"] == pytest.approx(1.0, abs=1e-9)
        assert bp["bearing_deg"] == pytest.approx(-90.0)

    def test_mounting_offset_270_sensor_0_becomes_body_270(self) -> None:
        """With mounting_offset=270, raw 0° → body 270° → to the left."""
        bp = lidar_point_to_body_frame(
            _pt(0.0, distance=1000), mounting_offset_deg=270.0
        )
        assert bp["x_m"] == pytest.approx(0.0, abs=1e-9)
        assert bp["y_m"] == pytest.approx(1.0, abs=1e-9)
        assert bp["bearing_deg"] == pytest.approx(-90.0)

    def test_mounting_offset_270_sensor_90_becomes_body_0(self) -> None:
        """With mounting_offset=270, raw 90° → body 0° → straight forward."""
        bp = lidar_point_to_body_frame(
            _pt(90.0, distance=1000), mounting_offset_deg=270.0
        )
        assert bp["x_m"] == pytest.approx(1.0, abs=1e-9)
        assert bp["y_m"] == pytest.approx(0.0, abs=1e-9)
        assert bp["bearing_deg"] == pytest.approx(0.0)

    def test_distance_mm_to_m_conversion(self) -> None:
        """2500 mm input → distance_m == 2.5."""
        bp = lidar_point_to_body_frame(_pt(0.0, distance=2500), mounting_offset_deg=0.0)
        assert bp["distance_m"] == pytest.approx(2.5)
        assert bp["x_m"] == pytest.approx(2.5)

    def test_zero_distance_preserves_zero(self) -> None:
        """Out-of-range point (distance=0) → distance_m == 0.0."""
        bp = lidar_point_to_body_frame(_pt(0.0, distance=0), mounting_offset_deg=0.0)
        assert bp["distance_m"] == pytest.approx(0.0)
        assert bp["x_m"] == pytest.approx(0.0)
        assert bp["y_m"] == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# Group 4 — filter_forward_arc
# ---------------------------------------------------------------------------


class TestFilterForwardArc:
    def test_straight_ahead_included(self) -> None:
        pts = [_body(0.0)]
        assert len(filter_forward_arc(pts, half_width_deg=30.0)) == 1

    def test_exactly_at_boundary_included(self) -> None:
        """±30° exactly is on the boundary and must be included."""
        pts = [_body(30.0), _body(-30.0)]
        assert len(filter_forward_arc(pts, half_width_deg=30.0)) == 2

    def test_just_outside_boundary_excluded(self) -> None:
        pts = [_body(30.1), _body(-30.1)]
        assert len(filter_forward_arc(pts, half_width_deg=30.0)) == 0

    def test_90_deg_excluded_from_30_arc(self) -> None:
        pts = [_body(90.0), _body(-90.0)]
        assert len(filter_forward_arc(pts, half_width_deg=30.0)) == 0

    def test_empty_list_returns_empty(self) -> None:
        assert filter_forward_arc([], half_width_deg=30.0) == []

    def test_custom_half_width(self) -> None:
        pts = [_body(45.0), _body(-45.0), _body(0.0)]
        assert len(filter_forward_arc(pts, half_width_deg=45.0)) == 3

    def test_wrap_boundary_negative_bearing_included(self) -> None:
        """A point at body-frame 350° has bearing −10° — inside a ±30° arc."""
        # body-frame 350° → wrap_180 = −10° (10° to the left)
        bp = lidar_point_to_body_frame(
            _pt(350.0, distance=1000), mounting_offset_deg=0.0
        )
        assert bp["bearing_deg"] == pytest.approx(-10.0, abs=1e-6)
        result = filter_forward_arc([bp], half_width_deg=30.0)
        assert len(result) == 1

    def test_wrap_boundary_point_outside_arc_excluded(self) -> None:
        """A point at body-frame 320° has bearing −40° — outside a ±30° arc."""
        bp = lidar_point_to_body_frame(
            _pt(320.0, distance=1000), mounting_offset_deg=0.0
        )
        assert bp["bearing_deg"] == pytest.approx(-40.0, abs=1e-6)
        result = filter_forward_arc([bp], half_width_deg=30.0)
        assert len(result) == 0


# ---------------------------------------------------------------------------
# Group 5 — nearest_forward_point
# ---------------------------------------------------------------------------


class TestNearestForwardPoint:
    def test_empty_list_returns_none(self) -> None:
        assert nearest_forward_point([]) is None

    def test_all_zero_distance_returns_none(self) -> None:
        pts = [_body(0.0, distance_m=0.0), _body(5.0, distance_m=0.0)]
        assert nearest_forward_point(pts) is None

    def test_single_valid_point_returned(self) -> None:
        pts = [_body(0.0, distance_m=1.5)]
        result = nearest_forward_point(pts)
        assert result is not None
        assert result["distance_m"] == pytest.approx(1.5)

    def test_nearest_selected_from_multiple(self) -> None:
        pts = [
            _body(5.0, distance_m=3.0),
            _body(0.0, distance_m=1.0),
            _body(-10.0, distance_m=2.0),
        ]
        result = nearest_forward_point(pts)
        assert result is not None
        assert result["distance_m"] == pytest.approx(1.0)

    def test_zero_distance_filtered_out_nearest_valid_selected(self) -> None:
        """Zero-distance (out-of-range) points are skipped."""
        pts = [
            _body(0.0, distance_m=0.0),
            _body(5.0, distance_m=2.5),
        ]
        result = nearest_forward_point(pts)
        assert result is not None
        assert result["distance_m"] == pytest.approx(2.5)
