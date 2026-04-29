"""Pan servo heading computation and command generation.

Converts a detection bounding-box centroid into an absolute pan servo command via:
  pixel_to_bearing_deg → tilt correction → deadband guard → clamp → set_pan_tilt

Pure helper functions are separated for direct unit testing.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
from loguru import logger

from ..utils.fisheye_utils import pixel_to_bearing_deg

if TYPE_CHECKING:
    from numpy.typing import NDArray


def apply_tilt_correction(heading_deg: float, tilt_deg: float) -> float:
    """Project heading error onto the horizontal plane given camera tilt.

    Parameters
    ----------
    heading_deg : float
        Raw horizontal heading error in degrees (camera optical frame).
    tilt_deg : float
        Camera tilt angle from horizontal in degrees.

    Returns
    -------
    float
        Corrected heading error in degrees.

    Examples
    --------
    >>> apply_tilt_correction(10.0, 0.0)
    10.0
    >>> import math
    >>> math.isclose(apply_tilt_correction(10.0, 90.0), 0.0, abs_tol=1e-9)
    True
    """
    return heading_deg * math.cos(math.radians(tilt_deg))


def within_deadband(heading_deg: float, pos_deg: float, neg_deg: float) -> bool:
    """Return True if heading error falls within the configured deadband.

    Parameters
    ----------
    heading_deg : float
        Heading error to test in degrees.
    pos_deg : float
        Upper deadband boundary (positive degrees).
    neg_deg : float
        Lower deadband boundary (negative degrees).

    Returns
    -------
    bool
        True when ``neg_deg <= heading_deg <= pos_deg``.

    Examples
    --------
    >>> within_deadband(3.0, 5.0, -5.0)
    True
    >>> within_deadband(6.0, 5.0, -5.0)
    False
    """
    return neg_deg <= heading_deg <= pos_deg


def clamp_pan(pan_deg: float, cmd_min_deg: float, cmd_max_deg: float) -> float:
    """Clamp a pan command to the configured mechanical limits.

    Parameters
    ----------
    pan_deg : float
        Desired pan command in degrees.
    cmd_min_deg : float
        Minimum allowable pan command in degrees.
    cmd_max_deg : float
        Maximum allowable pan command in degrees.

    Returns
    -------
    float
        Clamped pan command in degrees.

    Examples
    --------
    >>> clamp_pan(50.0, -45.0, 45.0)
    45.0
    >>> clamp_pan(-50.0, -45.0, 45.0)
    -45.0
    """
    return max(cmd_min_deg, min(cmd_max_deg, pan_deg))


class PanController:
    """Converts a bounding-box centroid to an absolute pan servo command.

    Maintains the last commanded pan position so the no-detection and hold
    paths both produce a hardware hold (no new command issued).

    The detection centroid is assumed to come from the Waveshare RGB fisheye
    camera whose intrinsics (K, D) are read from the ``waveshare_rgb`` block
    in ``sensor_config.yaml``.

    Three oscillation-reducing controls are applied in ``update()`` in this order:

    1. Proportional gain (``gain_kp``) scales the corrected heading error before
       accumulation to slow convergence and reduce overshoot.
    2. Hysteresis deadband uses a stateful enter/exit threshold pair so the
       controller cannot chatter at the deadband boundary.
    3. Delta clamp (``delta_max_deg``) caps the per-cycle command change to
       suppress large impulsive jumps from noisy detections.

    Parameters
    ----------
    K : NDArray[np.float64]
        3×3 fisheye camera matrix.
    D : NDArray[np.float64]
        Fisheye distortion coefficients shaped ``(4, 1)``.
    cmd_min_deg : float
        Minimum pan servo command in degrees.
    cmd_max_deg : float
        Maximum pan servo command in degrees.
    gain_kp : float
        Proportional gain applied to corrected heading error before accumulation.
        Values in (0, 1) slow convergence and reduce overshoot.
    delta_max_deg : float
        Maximum per-cycle pan command change in degrees (slew rate cap).
    hysteresis_enter_deg : float
        Enter-hold threshold in degrees; motion is suppressed when the scaled
        heading error magnitude falls at or below this value.
    hysteresis_exit_deg : float
        Exit-hold threshold in degrees; motion resumes only when the scaled
        heading error magnitude exceeds this value (must be >= hysteresis_enter_deg).
    tilt_deg : float
        Fixed camera tilt angle in degrees used for horizontal projection correction.
        Defaults to 0.0 (no correction). Hook for future dynamic tilt tracking.
    """

    def __init__(
        self,
        K: NDArray[np.float64],
        D: NDArray[np.float64],
        cmd_min_deg: float,
        cmd_max_deg: float,
        gain_kp: float,
        delta_max_deg: float,
        hysteresis_enter_deg: float,
        hysteresis_exit_deg: float,
        tilt_deg: float = 0.0,
    ) -> None:
        self._K = K
        self._D = D
        self._cmd_min_deg = cmd_min_deg
        self._cmd_max_deg = cmd_max_deg
        self._gain_kp = gain_kp
        self._delta_max_deg = delta_max_deg
        self._hysteresis_enter_deg = hysteresis_enter_deg
        self._hysteresis_exit_deg = hysteresis_exit_deg
        self._tilt_deg = tilt_deg
        self._current_pan_deg: float = 0.0
        self._in_hold: bool = False
        logger.debug(
            "PanController initialised "
            "(cmd=[{}, {}]°, kp={}, delta_max={:.1f}°, "
            "hysteresis=[enter={:.1f}°, exit={:.1f}°], tilt={:.1f}°).",
            cmd_min_deg,
            cmd_max_deg,
            gain_kp,
            delta_max_deg,
            hysteresis_enter_deg,
            hysteresis_exit_deg,
            tilt_deg,
        )

    @property
    def current_pan_deg(self) -> float:
        """Last commanded pan servo position in degrees."""
        return self._current_pan_deg

    def update(
        self,
        bbox_centre_u: float | None,
        bbox_centre_v: float | None,
    ) -> float | None:
        """Compute a pan command from a detection centroid.

        Returns the new absolute pan command in degrees, or ``None`` when the
        pan position should be held (no detection or hysteresis hold active).

        Parameters
        ----------
        bbox_centre_u : float | None
            Horizontal pixel coordinate of the bounding-box centroid, or
            ``None`` when there is no valid detection.
        bbox_centre_v : float | None
            Vertical pixel coordinate of the bounding-box centroid, or
            ``None`` when there is no valid detection.

        Returns
        -------
        float | None
            Clamped absolute pan command in degrees, or ``None`` to hold.
        """
        if bbox_centre_u is None or bbox_centre_v is None:
            logger.debug(
                "Pan: no detection — holding at {:.2f}°.", self._current_pan_deg
            )
            return None

        heading_deg = pixel_to_bearing_deg(
            bbox_centre_u, bbox_centre_v, self._K, self._D
        )
        corrected = apply_tilt_correction(heading_deg, self._tilt_deg)

        # 1. Proportional gain: scale heading error before accumulation.
        scaled = self._gain_kp * corrected

        # 2. Hysteresis deadband: stateful enter/exit threshold guard.
        if self._in_hold:
            if not within_deadband(
                scaled, self._hysteresis_exit_deg, -self._hysteresis_exit_deg
            ):
                self._in_hold = False
            else:
                logger.debug(
                    "Pan: in hold, |scaled|={:.2f}° < exit={:.1f}° — holding.",
                    abs(scaled),
                    self._hysteresis_exit_deg,
                )
                return None
        else:
            if within_deadband(
                scaled, self._hysteresis_enter_deg, -self._hysteresis_enter_deg
            ):
                self._in_hold = True
                logger.debug(
                    "Pan: entering hold, |scaled|={:.2f}° <= enter={:.1f}°.",
                    abs(scaled),
                    self._hysteresis_enter_deg,
                )
                return None

        # 3. Delta clamp: cap per-cycle command change (slew rate limit).
        delta = max(-self._delta_max_deg, min(self._delta_max_deg, scaled))
        target_pan = self._current_pan_deg + delta
        new_pan = clamp_pan(target_pan, self._cmd_min_deg, self._cmd_max_deg)
        self._current_pan_deg = new_pan
        logger.debug(
            "Pan: corrected={:.2f}°, scaled={:.2f}°, delta={:.2f}°, pan_cmd={:.2f}°.",
            corrected,
            scaled,
            delta,
            new_pan,
        )
        return new_pan
