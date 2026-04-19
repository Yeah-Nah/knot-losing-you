"""Fisheye camera geometry utilities for the Waveshare RGB camera.

Provides pixel-to-bearing conversion using OpenCV's fisheye model and the
intrinsic calibration stored in ``sensor_config.yaml``.

These pure functions are used by:

- ``calibrate_ugv_drive.py``  (Phase 2): rover rotation angle measurement
- Future Phase 3 runtime: bounding-box pixel centroid → angular heading error

Most functions are pure.  ``undistort_frame`` maintains a module-level remap
cache so that ``cv2.fisheye.initUndistortRectifyMap`` is called at most once
per unique (K, D, frame-size) combination.
"""

from __future__ import annotations

import math
from typing import Any

import cv2
import numpy as np

# Cache of pre-computed fisheye undistortion maps keyed by (K_bytes, D_bytes, (h, w)).
# Built on first use; avoids recomputing O(W×H) maps on every frame.
_UNDISTORT_MAP_CACHE: dict[
    tuple[bytes, bytes, tuple[int, int]], tuple[np.ndarray, np.ndarray]
] = {}


def load_fisheye_intrinsics(
    sensor_cfg: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray]:
    """Return (K, D) from the ``waveshare_rgb`` block of sensor_config.

    Parameters
    ----------
    sensor_cfg : dict[str, Any]
        Parsed contents of ``sensor_config.yaml``.

    Returns
    -------
    K : np.ndarray
        3×3 camera matrix (float64), row-major as stored in YAML.
    D : np.ndarray
        Fisheye distortion coefficients shaped ``(4, 1)`` (float64).

    Raises
    ------
    ValueError
        If the ``waveshare_rgb`` section, ``camera_matrix``, or
        ``dist_coeffs`` is absent or null.

    Examples
    --------
    >>> cfg = {
    ...     "waveshare_rgb": {
    ...         "camera_matrix": [[500, 0, 320], [0, 500, 240], [0, 0, 1]],
    ...         "dist_coeffs": [0, 0, 0, 0],
    ...     }
    ... }
    >>> K, D = load_fisheye_intrinsics(cfg)
    >>> K.shape
    (3, 3)
    >>> D.shape
    (4, 1)
    """
    ws: dict[str, Any] = dict(sensor_cfg.get("waveshare_rgb") or {})
    matrix = ws.get("camera_matrix")
    if matrix is None:
        raise ValueError(
            "waveshare_rgb.camera_matrix is null in sensor_config.yaml. "
            "Run calibrate_waveshare_camera.py first."
        )
    dist = ws.get("dist_coeffs")
    if dist is None:
        raise ValueError(
            "waveshare_rgb.dist_coeffs is null in sensor_config.yaml. "
            "Run calibrate_waveshare_camera.py first."
        )
    K = np.array(matrix, dtype=np.float64)
    D = np.array(dist, dtype=np.float64).flatten().reshape((4, 1))
    return K, D


def pixel_to_normalised(
    u: float,
    v: float,
    K: np.ndarray,
    D: np.ndarray,
) -> tuple[float, float]:
    """Undistort a pixel to normalised camera coordinates using the fisheye model.

    Applies ``cv2.fisheye.undistortPoints`` with no additional rectification
    (``R=None``, ``P=None``).  The returned normalised coordinates correspond
    to the unit image plane (z = 1).

    Parameters
    ----------
    u : float
        Horizontal pixel coordinate (column).
    v : float
        Vertical pixel coordinate (row).
    K : np.ndarray
        3×3 fisheye camera matrix.
    D : np.ndarray
        Fisheye distortion coefficients shaped ``(4, 1)`` — [k1, k2, k3, k4].

    Returns
    -------
    (x_n, y_n) : tuple[float, float]
        Undistorted normalised camera coordinates.  Positive ``x_n`` means the
        point is to the right of the optical axis.

    Examples
    --------
    >>> K = np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]])
    >>> D = np.zeros((4, 1))
    >>> x_n, y_n = pixel_to_normalised(320., 240., K, D)
    >>> abs(x_n) < 1e-9 and abs(y_n) < 1e-9
    True
    """
    pts = np.array([[[u, v]]], dtype=np.float64)
    normalised = cv2.fisheye.undistortPoints(pts, K, D)
    x_n = float(normalised[0, 0, 0])
    y_n = float(normalised[0, 0, 1])
    return x_n, y_n


def pixel_to_bearing_deg(
    u: float,
    v: float,
    K: np.ndarray,
    D: np.ndarray,
) -> float:
    """Return the horizontal bearing to pixel (u, v) in degrees.

    Converts the distorted pixel to normalised camera coordinates via the
    fisheye model, then computes the horizontal bearing as
    ``degrees(atan(x_n))``.

    Sign convention: **positive bearing = target to the RIGHT of the optical
    axis**.  For rover drive calibration, a counter-clockwise (CCW) rover
    rotation causes the stationary target to drift rightward in the image, so
    the bearing increases and ``Δθ = θ_after − θ_before > 0`` for a positive
    (CCW) angular velocity command.

    Parameters
    ----------
    u : float
        Horizontal pixel coordinate (column).
    v : float
        Vertical pixel coordinate (row).  Required for accurate fisheye
        undistortion — the radially-symmetric distortion model couples (u, v).
    K : np.ndarray
        3×3 fisheye camera matrix.
    D : np.ndarray
        Fisheye distortion coefficients shaped ``(4, 1)``.

    Returns
    -------
    float
        Horizontal bearing in degrees.

    Examples
    --------
    >>> K = np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]])
    >>> D = np.zeros((4, 1))
    >>> pixel_to_bearing_deg(320., 240., K, D)   # centre → 0°
    0.0
    >>> pixel_to_bearing_deg(820., 240., K, D)   # 500 px right → atan(1) = 45°
    45.0
    """
    x_n, _y_n = pixel_to_normalised(u, v, K, D)
    return math.degrees(math.atan(x_n))


def undistort_frame(
    frame: cv2.typing.MatLike,
    K: np.ndarray,
    D: np.ndarray,
) -> cv2.typing.MatLike:
    """Undistort a fisheye frame using cached remap maps.

    Builds ``cv2.fisheye.initUndistortRectifyMap`` maps once per unique
    (K, D, frame-size) combination and caches them.  Subsequent calls with
    the same parameters use ``cv2.remap`` directly, avoiding the O(W×H)
    map-rebuild cost on every frame.

    The output maps to a virtual pinhole camera with the same principal point
    and focal length as the original (``Knew=K``).  Pixels in the returned
    image satisfy the pinhole relation ``x_n = (u − cx) / fx``.

    Parameters
    ----------
    frame : cv2.typing.MatLike
        Raw distorted input frame.
    K : np.ndarray
        3×3 fisheye camera matrix.
    D : np.ndarray
        Fisheye distortion coefficients shaped ``(4, 1)``.

    Returns
    -------
    cv2.typing.MatLike
        Undistorted frame with the same resolution as the input.
    """
    h, w = frame.shape[:2]
    cache_key = (K.tobytes(), D.tobytes(), (h, w))
    if cache_key not in _UNDISTORT_MAP_CACHE:
        map_x, map_y = cv2.fisheye.initUndistortRectifyMap(
            K, D, np.eye(3), K, (w, h), cv2.CV_32F
        )
        _UNDISTORT_MAP_CACHE[cache_key] = (map_x, map_y)
    map_x, map_y = _UNDISTORT_MAP_CACHE[cache_key]
    return cv2.remap(frame, map_x, map_y, cv2.INTER_LINEAR)


def pixel_to_bearing_deg_pinhole(u: float, cx: float, fx: float) -> float:
    """Return the horizontal bearing to an undistorted-frame pixel in degrees.

    For a frame undistorted with ``Knew=K`` the pinhole relation holds:
    ``x_n = (u − cx) / fx``, giving ``bearing = atan(x_n)``.

    Sign convention matches ``pixel_to_bearing_deg``: positive bearing means
    the target is to the right of the optical axis.

    Parameters
    ----------
    u : float
        Horizontal pixel coordinate in the **undistorted** frame.
    cx : float
        Principal point x-coordinate (``K[0, 2]``).
    fx : float
        Horizontal focal length (``K[0, 0]``).

    Returns
    -------
    float
        Horizontal bearing in degrees.

    Examples
    --------
    >>> pixel_to_bearing_deg_pinhole(320.0, 320.0, 500.0)   # centre → 0°
    0.0
    >>> pixel_to_bearing_deg_pinhole(820.0, 320.0, 500.0)   # cx + fx → 45°
    45.0
    """
    return math.degrees(math.atan((u - cx) / fx))
