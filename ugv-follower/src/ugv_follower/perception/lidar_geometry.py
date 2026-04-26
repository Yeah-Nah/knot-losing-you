"""LiDAR coordinate-system conversion for the LDRobot D500.

Converts raw sensor polar coordinates into rover body-frame Cartesian
coordinates and provides forward-arc filtering used by the Phase 3
control pipeline.

Angle conventions
-----------------
- D500 sensor frame: 0° = sensor forward, clockwise-positive, [0, 360)
- Rover body frame:  0° = rover forward, clockwise-positive,
                     +x = forward, +y = left
- mounting_offset_deg (270): direction the LiDAR 0° axis points in the
  rover body frame (i.e. the sensor is mounted rotated 270° from rover
  forward).
- Bearing sign convention matches ``fisheye_utils.pixel_to_bearing_deg``:
  **positive bearing = target to the RIGHT of the forward axis**.

These pure functions are used by:

- ``pipeline.py`` (Phase 3): forward-obstacle ranging on each control tick
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, TypedDict

if TYPE_CHECKING:
    from ugv_follower.perception.lidar_access import LidarPoint


class BodyFramePoint(TypedDict):
    """A LiDAR point expressed in rover body-frame coordinates.

    All distances are in metres.  The bearing convention is consistent
    with ``fisheye_utils.pixel_to_bearing_deg``: positive = right.
    """

    x_m: float
    """Forward component (metres); positive = forward."""
    y_m: float
    """Lateral component (metres); positive = left."""
    distance_m: float
    """Slant range (metres); 0 means out-of-range."""
    bearing_deg: float
    """Signed horizontal bearing (degrees); positive = right of forward axis."""


def wrap_360(angle_deg: float) -> float:
    """Wrap *angle_deg* to the half-open interval ``[0, 360)``.

    Parameters
    ----------
    angle_deg : float
        Angle in degrees, any value.

    Returns
    -------
    float
        Equivalent angle in ``[0, 360)``.

    Examples
    --------
    >>> wrap_360(0.0)
    0.0
    >>> wrap_360(360.0)
    0.0
    >>> wrap_360(-90.0)
    270.0
    >>> wrap_360(720.0)
    0.0
    """
    return angle_deg % 360.0


def wrap_180(angle_deg: float) -> float:
    """Wrap *angle_deg* to the half-open interval ``(-180, 180]``.

    Parameters
    ----------
    angle_deg : float
        Angle in degrees, any value.

    Returns
    -------
    float
        Equivalent angle in ``(-180, 180]``.

    Examples
    --------
    >>> wrap_180(0.0)
    0.0
    >>> wrap_180(180.0)
    180.0
    >>> wrap_180(270.0)
    -90.0
    >>> wrap_180(-90.0)
    -90.0
    >>> wrap_180(181.0)
    -179.0
    """
    a = angle_deg % 360.0
    return a if a <= 180.0 else a - 360.0


def lidar_point_to_body_frame(
    point: LidarPoint,
    mounting_offset_deg: float,
    forward_displacement_m: float = 0.0,
) -> BodyFramePoint:
    """Convert a single D500 polar point to rover body-frame Cartesian.

    Applies the mounting offset so that 0° in the output corresponds to
    rover forward, then projects to (x, y) using the body-frame convention
    (+x forward, +y left, clockwise-positive angles).  If the LiDAR origin
    is not coincident with the rover body centre, ``forward_displacement_m``
    translates the point from LiDAR-origin frame to rover-centre frame.

    Parameters
    ----------
    point : LidarPoint
        Raw measurement from ``LidarAccess.get_scan()``.
        ``distance`` is in millimetres; 0 means out-of-range.
    mounting_offset_deg : float
        Direction the LiDAR 0° axis points in the rover body frame
        (degrees).  Read from ``Settings.lidar_mounting_offset_deg``.
    forward_displacement_m : float, optional
        Forward displacement of the LiDAR origin from the rover body centre
        (metres).  Positive = LiDAR is ahead of centre.  Default is 0.0
        (LiDAR at body centre).  Read from
        ``Settings.lidar_forward_displacement_m``.

    Returns
    -------
    BodyFramePoint
        Body-frame representation of *point*, expressed relative to the
        rover body centre.  ``distance_m`` is the slant range in metres.
        ``bearing_deg`` is positive to the right.

    Examples
    --------
    >>> pt: LidarPoint = {"angle": 0.0, "distance": 1000, "intensity": 100}
    >>> bp = lidar_point_to_body_frame(pt, mounting_offset_deg=0.0)
    >>> round(bp["x_m"], 6), round(bp["y_m"], 6)
    (1.0, -0.0)
    """
    theta_body = wrap_360(point["angle"] + mounting_offset_deg)
    theta_rad = math.radians(theta_body)
    raw_distance_m = point["distance"] / 1000.0
    x_m = raw_distance_m * math.cos(theta_rad)
    # CW-positive with +y=left requires negating the sin component
    y_m = -raw_distance_m * math.sin(theta_rad)
    # Translate from LiDAR-origin frame to rover-centre frame.
    x_m += forward_displacement_m
    distance_m = math.hypot(x_m, y_m)
    bearing_deg = wrap_180(math.degrees(math.atan2(-y_m, x_m)))
    return {
        "x_m": x_m,
        "y_m": y_m,
        "distance_m": distance_m,
        "bearing_deg": bearing_deg,
    }


def filter_forward_arc(
    points: list[BodyFramePoint],
    half_width_deg: float = 30.0,
) -> list[BodyFramePoint]:
    """Return only the points whose bearing falls within the forward arc.

    Uses the signed ``bearing_deg`` field so the wrap boundary at 0°/360°
    is handled transparently — no special-casing needed.

    Parameters
    ----------
    points : list[BodyFramePoint]
        Body-frame points, e.g. from a batch of ``lidar_point_to_body_frame``
        calls.
    half_width_deg : float, optional
        Half-angle of the forward arc in degrees.  Default is 30°, giving
        a ±30° (60° total) forward cone.

    Returns
    -------
    list[BodyFramePoint]
        Subset of *points* whose ``|bearing_deg| <= half_width_deg``.

    Examples
    --------
    >>> pts = [{"x_m": 1.0, "y_m": 0.0, "distance_m": 1.0, "bearing_deg": 0.0},
    ...        {"x_m": 0.0, "y_m": -1.0, "distance_m": 1.0, "bearing_deg": 90.0}]
    >>> len(filter_forward_arc(pts, half_width_deg=30.0))
    1
    """
    return [p for p in points if abs(p["bearing_deg"]) <= half_width_deg]


def nearest_forward_point(
    points: list[BodyFramePoint],
) -> BodyFramePoint | None:
    """Return the closest valid point from a forward-arc list.

    Points with ``distance_m == 0`` are treated as out-of-range and
    excluded.

    Parameters
    ----------
    points : list[BodyFramePoint]
        Forward-arc points, typically the output of ``filter_forward_arc``.

    Returns
    -------
    BodyFramePoint | None
        The point with the smallest positive ``distance_m``, or ``None``
        if *points* is empty or all distances are zero.

    Examples
    --------
    >>> pts = [
    ...     {"x_m": 2.0, "y_m": 0.0, "distance_m": 2.0, "bearing_deg": 0.0},
    ...     {"x_m": 1.0, "y_m": 0.0, "distance_m": 1.0, "bearing_deg": 0.0},
    ... ]
    >>> nearest_forward_point(pts)["distance_m"]
    1.0
    """
    valid = [p for p in points if p["distance_m"] > 0.0]
    if not valid:
        return None
    return min(valid, key=lambda p: p["distance_m"])
