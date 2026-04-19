"""Rover drive calibration for the Waveshare UGV Rover 6WD.

Measures the rover's actual turn-rate response to commanded angular velocity
and identifies the angular dead-band.  Angle measurement uses the Waveshare
RGB fisheye camera and the intrinsic calibration already stored in
``sensor_config.yaml``.

Measurement approach
--------------------
The rover is commanded to rotate in place (``linear = 0``) while a stationary
target is tracked in the camera frame via template matching.  Bearing to the
target before and after each rotation is computed using
``cv2.fisheye.undistortPoints`` with the stored fisheye intrinsics, giving an
accurate angle estimate even for large target displacements from the optical
axis.  A forward-offset correction (same geometry as pan-tilt calibration)
removes the bias introduced by the camera lens being displaced forward from
the rover's differential-drive rotation centre.

The sweep runs four directional blocks in order: gain CCW, gain CW,
dead-band CCW, dead-band CW.  The operator is prompted to re-centre the
rover between every block, avoiding directional instability from sign
reversals and accumulated drift.

Results written to ``sensor_config.yaml`` under the ``ugv_drive`` key:

- ``effective_track_width_m`` — corrects the differential-drive kinematic model
- ``turn_rate_gain`` — k = ω_actual / ω_commanded
- ``angular_dead_band_rad_s`` — minimum |ω| that overcomes static friction

Must be run on the Raspberry Pi with the UGV rover and Waveshare RGB camera
connected.  Intrinsic calibration must have been completed first.

The ``ugv-follower`` package must be installed before running::

    pip install -e ugv-follower/

How to run
----------
::

    ugv-calibrate-drive

    # Offline refit from an existing CSV (no hardware required):
    ugv-calibrate-drive --replay calibration/ugv_drive/<timestamp>.csv
    ugv-calibrate-drive --replay calibration/ugv_drive/<timestamp>.csv --save

Arguments
---------
--replay        Path to an existing raw CSV for offline refit (no hardware).
--save          When used with --replay, write results to sensor_config.yaml.
--camera-device V4L2 device path (overrides calibration_config.yaml).
--sensor-config Path to sensor_config.yaml (default: configs/sensor_config.yaml).
--cal-config    Path to calibration_config.yaml (default: configs/calibration_config.yaml).
--noise-floor   Override ugv_drive.noise_floor_deg from calibration config.

Hardware setup
--------------
1. Place the rover on a flat surface with ≥ 2 m clear space in all directions.
2. Connect the UGV rover sub-controller and Waveshare RGB camera (USB).
3. Place a high-contrast stationary target (e.g. tape cross, printed pattern)
   approximately ``target_distance_m`` in front of the rover.
4. Power on the rover.

Calibration procedure
---------------------
1. Run the script.
2. Open ``http://<pi-ip>:8080/stream`` in a browser.
3. Verify the target is visible and centred on the crosshair at ``cx``.
4. ``GET /confirm`` to start the sweep.
5. Poll ``GET /status`` until ``state`` is ``COMPLETE`` (or ``FAILED``).
6. ``GET /save`` to write results to sensor_config.yaml.
7. ``GET /abort`` at any time to cancel without saving.

Endpoints
---------
GET /stream   — MJPEG stream with crosshair guide and state overlay.
GET /status   — JSON: {``state``, ``progress_pct``, ``current_step``,
                       ``total_steps``, ``error_message``}.
GET /confirm  — Start the sweep (only valid in WAITING_TARGET).
GET /abort    — Cancel and shut down.
GET /save     — Write CSV and sensor_config.yaml (only valid in COMPLETE).
GET /reset    — Reset back to WAITING_TARGET (not valid while SWEEPING).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Sequence

import cv2
import numpy as np
import yaml
from loguru import logger

from ugv_follower.control.ugv_controller import UGVController
from ugv_follower.utils.camera_preflight import ensure_camera_device_available
from ugv_follower.utils.config_utils import get_project_root
from ugv_follower.utils.fisheye_utils import (
    load_fisheye_intrinsics,
    pixel_to_bearing_deg,
)

_DEFAULT_SENSOR_CONFIG: Path = get_project_root() / "configs" / "sensor_config.yaml"
_DEFAULT_CAL_CONFIG: Path = get_project_root() / "configs" / "calibration_config.yaml"
_CSV_DIR: Path = get_project_root() / "calibration" / "ugv_drive"

_STREAM_PORT: int = 8080
_JPEG_QUALITY: int = 80

# Pixels from frame edge below which a centroid is flagged as near-edge.
_EDGE_MARGIN_PX: int = 20

# Seconds to wait after connecting the UGV before capturing the first frame.
_CONNECT_SETTLE_S: float = 1.0

# Run types stored in the CSV.
_GAIN_RUN: str = "gain_sweep"
_DEAD_RUN: str = "dead_band"

# When |k_pos − k_neg| / max(k_pos, k_neg) exceeds this, asymmetric gains are reported.
_ASYMMETRY_THRESHOLD: float = 0.08

# Quality flag bit for a near-stall gain step (|Δ_corrected| too small to be a valid measurement).
_STALL_FLAG: int = 4
# Minimum fraction of expected rotation that corrected delta must reach to avoid the stall gate.
_MIN_ROTATION_FRACTION: float = 0.10

# Canonical CSV column order.
_CSV_COLUMNS: list[str] = [
    "timestamp_s",
    "run_type",
    "omega_commanded_rad_s",
    "direction",
    "command_duration_s",
    "bearing_before_deg",
    "bearing_after_deg",
    "delta_bearing_deg",
    "corrected_delta_deg",  # gain_sweep only; empty string for dead_band rows
    "expected_angle_deg",  # gain_sweep only; empty string for dead_band rows
    "moved",  # dead_band only ("0"/"1"); empty string for gain_sweep rows
    "match_score_before",
    "match_score_after",
    "quality_flag",
]

_CSV_INT_COLS: frozenset[str] = frozenset({"quality_flag"})
_CSV_STR_COLS: frozenset[str] = frozenset({"run_type", "direction"})
_CSV_OPTIONAL_FLOAT_COLS: frozenset[str] = frozenset(
    {"corrected_delta_deg", "expected_angle_deg"}
)
_CSV_BOOL_COLS: frozenset[str] = frozenset({"moved"})


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------


class CalibrationState(str, Enum):
    """State machine values for the calibration workflow."""

    WAITING_TARGET = "WAITING_TARGET"
    SWEEPING = "SWEEPING"
    WAITING_RECENTER = "WAITING_RECENTER"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"
    ABORTED = "ABORTED"


# ---------------------------------------------------------------------------
# Data models
# ---------------------------------------------------------------------------


@dataclass
class SweepStatus:
    """Snapshot of sweep progress, treated as an immutable value."""

    state: CalibrationState = CalibrationState.WAITING_TARGET
    progress_pct: float = 0.0
    current_step: int = 0
    total_steps: int = 0
    error_message: str | None = None


@dataclass(frozen=True)
class DriveCalConfig:
    """Validated, derived configuration for one calibration run.

    Constructed by ``_load_config()`` from sensor_config.yaml,
    calibration_config.yaml, and parsed CLI arguments.
    """

    # From sensor_config.yaml — waveshare_rgb
    K: np.ndarray
    D: np.ndarray
    frame_width: int
    frame_height: int
    cx: float  # principal point x — used for crosshair overlay

    # From sensor_config.yaml — ugv
    ugv_port: str
    ugv_baud_rate: int
    ugv_chassis_main: int
    ugv_chassis_module: int
    ugv_track_width_nom: float

    # From calibration_config.yaml — ugv_drive
    omega_commands_rad_s: tuple[float, ...]
    command_duration_s: float
    settle_time_s: float
    dead_band_omega_steps_rad_s: tuple[float, ...]
    dead_band_duration_s: float
    dead_band_settle_s: float
    frames_to_average: int
    noise_floor_deg: float
    camera_offset_m: float
    target_distance_m: float
    template_half_width_px: int
    min_match_score: float
    camera_device: str
    calibration_surface: str

    sensor_config_path: Path


# ---------------------------------------------------------------------------
# Thread-safe state container
# ---------------------------------------------------------------------------


class CalibrationStateContainer:
    """Thread-safe container for sweep progress and the latest annotated frame."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._status: SweepStatus = SweepStatus()
        self._annotated_frame: cv2.typing.MatLike | None = None
        self._sweep_rows: list[dict[str, Any]] | None = None

    def update_frame(self, frame: cv2.typing.MatLike) -> None:
        """Replace the stored annotated frame.  Called from the frame thread."""
        with self._lock:
            self._annotated_frame = frame

    def get_annotated_jpeg(self) -> bytes | None:
        """JPEG-encode the latest annotated frame and return bytes, or None."""
        with self._lock:
            frame = self._annotated_frame
        if frame is None:
            return None
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, _JPEG_QUALITY])
        return bytes(buf) if ok else None

    def get_status(self) -> dict[str, Any]:
        """Return a JSON-serialisable snapshot of current state values."""
        with self._lock:
            s = self._status
        return {
            "state": s.state.value,
            "progress_pct": round(s.progress_pct, 1),
            "current_step": s.current_step,
            "total_steps": s.total_steps,
            "error_message": s.error_message,
        }

    def get_snapshot(self) -> SweepStatus:
        """Return a copy of the current ``SweepStatus``."""
        with self._lock:
            s = self._status
        return SweepStatus(
            state=s.state,
            progress_pct=s.progress_pct,
            current_step=s.current_step,
            total_steps=s.total_steps,
            error_message=s.error_message,
        )

    def try_start_sweep(self) -> bool:
        """Atomically transition WAITING_TARGET → SWEEPING.  Returns True on success."""
        with self._lock:
            if self._status.state != CalibrationState.WAITING_TARGET:
                return False
            self._status = SweepStatus(state=CalibrationState.SWEEPING)
            return True

    def try_start_recenter_resume(self) -> bool:
        """Atomically transition WAITING_RECENTER → SWEEPING, preserving progress.

        Returns
        -------
        bool
            True if the transition succeeded; False if not in WAITING_RECENTER.
        """
        with self._lock:
            if self._status.state != CalibrationState.WAITING_RECENTER:
                return False
            self._status = SweepStatus(
                state=CalibrationState.SWEEPING,
                progress_pct=self._status.progress_pct,
                current_step=self._status.current_step,
                total_steps=self._status.total_steps,
            )
            return True

    def update_sweep_progress(self, current_step: int, total_steps: int) -> None:
        """Update progress fields while remaining in SWEEPING state."""
        with self._lock:
            if self._status.state == CalibrationState.SWEEPING:
                pct = 100.0 * current_step / total_steps if total_steps else 0.0
                self._status = SweepStatus(
                    state=CalibrationState.SWEEPING,
                    progress_pct=pct,
                    current_step=current_step,
                    total_steps=total_steps,
                )

    def transition_to(self, new_status: SweepStatus) -> None:
        """Replace the current status atomically."""
        with self._lock:
            self._status = new_status

    def try_reset(self) -> bool:
        """Atomically reset to WAITING_TARGET, clearing previous results.

        Returns True on success, False if the sweep thread is still active
        (SWEEPING or WAITING_RECENTER).
        """
        with self._lock:
            if self._status.state in (
                CalibrationState.SWEEPING,
                CalibrationState.WAITING_RECENTER,
            ):
                return False
            self._status = SweepStatus()
            self._sweep_rows = None
            return True

    def set_sweep_rows(self, rows: list[dict[str, Any]]) -> None:
        """Store raw sweep rows after a successful sweep.  Called from sweep thread."""
        with self._lock:
            self._sweep_rows = rows

    def get_sweep_rows(self) -> list[dict[str, Any]] | None:
        """Return stored rows (None if sweep not yet complete)."""
        with self._lock:
            return self._sweep_rows


# ---------------------------------------------------------------------------
# Pure math helpers (module-level so tests can import them directly)
# ---------------------------------------------------------------------------


def correct_for_camera_offset(
    delta_bearing_deg: float, d_m: float, D_m: float
) -> float:
    """Remove the forward-offset bias from an observed bearing change.

    The camera lens is displaced forward from the rover's differential-drive
    rotation centre by ``d_m`` metres.  This makes the apparent bearing change
    (as seen by the camera) slightly larger than the true rover body rotation.
    The correction is::

        delta_corrected = delta_bearing − arcsin( d/D · sin(delta_bearing) )

    which is identical in form to ``correct_for_forward_offset`` in the
    pan-tilt calibration.  Setting ``d_m = 0.0`` returns ``delta_bearing``
    unchanged (naive model).

    Parameters
    ----------
    delta_bearing_deg : float
        Observed bearing change in degrees (θ_after − θ_before).
    d_m : float
        Forward offset of the camera lens from the rover rotation centre
        (metres).  Must be ``>= 0`` and ``< D_m``.
    D_m : float
        Distance from the rover rotation centre to the calibration target
        (metres).  Must be ``> 0``.

    Returns
    -------
    float
        Corrected rover heading change in degrees.

    Raises
    ------
    ValueError
        If the arcsin domain constraint is violated (only possible when
        ``d_m >= D_m``, which ``_validate_geometry`` prevents at startup).
    """
    if d_m == 0.0:
        return delta_bearing_deg
    delta_rad = math.radians(delta_bearing_deg)
    ratio = (d_m / D_m) * math.sin(delta_rad)
    if abs(ratio) >= 1.0:
        raise ValueError(
            f"Trig domain violation in correct_for_camera_offset: "
            f"d/D*sin(delta) = {ratio:.4f} must be in (-1, 1). "
            f"d={d_m}, D={D_m}, delta={delta_bearing_deg:.2f}°"
        )
    return delta_bearing_deg - math.degrees(math.asin(ratio))


def quality_flag(
    match_score: float,
    u_px: float,
    frame_width: int,
    min_match_score: float,
    edge_margin_px: int = _EDGE_MARGIN_PX,
) -> int:
    """Compute an integer quality flag for a single bearing measurement.

    Parameters
    ----------
    match_score : float
        ``cv2.matchTemplate`` (TM_CCOEFF_NORMED) score, 0–1.
    u_px : float
        Measured horizontal centroid in pixels.
    frame_width : int
        Frame width in pixels.
    min_match_score : float
        Score threshold below which ``low_match`` is True.
    edge_margin_px : int
        Pixels from frame edge below which ``near_edge`` is True.

    Returns
    -------
    int
        ``0`` = good, ``1`` = low_match, ``2`` = near_edge, ``3`` = both.
        Flags are bit-packed so that ``flag_a | flag_b`` combines two
        independent measurements into a single worst-case flag.
    """
    low_match = match_score < min_match_score
    near_edge = u_px < edge_margin_px or u_px > (frame_width - edge_margin_px)
    if low_match and near_edge:
        return 3
    if near_edge:
        return 2
    if low_match:
        return 1
    return 0


def _sign_consistent(corrected_delta_deg: float, expected_angle_deg: float) -> bool:
    """Return True if corrected delta and expected angle share the same sign.

    A sign mismatch indicates the rover rotated opposite to the commanded
    direction — typically caused by directional instability near the motion
    threshold.  Such samples corrupt the OLS gain fit and are excluded.

    Parameters
    ----------
    corrected_delta_deg : float
        Camera-offset-corrected bearing change (degrees).
    expected_angle_deg : float
        Kinematically expected bearing change (degrees).

    Returns
    -------
    bool
        ``True`` when both values are positive or both are negative.
        ``False`` when the signs differ or either value is zero.
    """
    return corrected_delta_deg * expected_angle_deg > 0.0


def fit_turn_rate_gain(
    corrected_deltas_deg: list[float],
    expected_angles_deg: list[float],
) -> dict[str, float]:
    """Fit the turn-rate gain k using OLS regression through the origin.

    The model is::

        delta_actual = k · delta_expected + ε

    with no intercept term (at zero commanded rotation, zero actual rotation
    is expected).  The least-squares estimate is::

        k = Σ(delta_i · expected_i) / Σ(expected_i²)

    Parameters
    ----------
    corrected_deltas_deg : list[float]
        Corrected actual bearing changes (degrees), including both CCW
        (positive) and CW (negative) samples.
    expected_angles_deg : list[float]
        Corresponding expected angles from the kinematic model
        (``± degrees(ω · T)``).

    Returns
    -------
    dict
        Keys: ``slope`` (= k), ``mae_deg``, ``max_abs_error_deg``.

    Raises
    ------
    ValueError
        If fewer than 2 samples are provided.
    """
    if len(corrected_deltas_deg) < 2:
        raise ValueError(
            f"fit_turn_rate_gain requires at least 2 samples, "
            f"got {len(corrected_deltas_deg)}."
        )
    deltas = np.array(corrected_deltas_deg, dtype=float)
    expected = np.array(expected_angles_deg, dtype=float)
    denom = float(np.dot(expected, expected))
    if denom == 0.0:
        raise ValueError("All expected angles are zero — cannot fit gain.")
    k = float(np.dot(deltas, expected)) / denom
    residuals = np.abs(deltas - k * expected)
    return {
        "slope": k,
        "mae_deg": float(np.mean(residuals)),
        "max_abs_error_deg": float(np.max(residuals)),
    }


def estimate_dead_band(
    omegas_rad_s: list[float],
    moved: list[bool],
) -> float | None:
    """Return the midpoint between the smallest moving and largest stalled ω.

    Parameters
    ----------
    omegas_rad_s : list[float]
        Probed angular velocities, sorted **descending** (largest first).
        All values must be positive.
    moved : list[bool]
        Whether the rover rotated at each corresponding ω (``True`` = rotated).

    Returns
    -------
    float or None
        Estimated dead-band threshold in rad/s.

        - Returns ``None`` if all probed ω values produced rotation (no
          dead-band detected within the tested range).
        - Returns the largest tested ω if all probed ω values stalled
          (conservative estimate: dead-band ≥ max tested ω).
        - Returns the midpoint between the last moving ω and the first
          stalled ω when a clear crossover is found.
    """
    if not omegas_rad_s:
        return None

    last_moving: float | None = None
    first_stalled: float | None = None

    for omega, did_move in zip(omegas_rad_s, moved):
        if did_move:
            last_moving = omega
        elif last_moving is not None:
            first_stalled = omega
            break

    if last_moving is None:
        # All stalled — dead-band is at least as large as the max tested ω.
        return float(omegas_rad_s[0])
    if first_stalled is None:
        # All moved — no dead-band detected in the tested range.
        return None
    return (float(last_moving) + float(first_stalled)) / 2.0


def analyse_runs(
    rows: list[dict[str, Any]],
    config: DriveCalConfig,
) -> dict[str, Any]:
    """Produce the ``ugv_drive`` sensor_config.yaml section from raw CSV rows.

    Steps:

    1. Filter gain_sweep rows to quality_flag == 0.
    2. Fit k (all), k_pos (CCW only), k_neg (CW only) via OLS through origin.
    3. Compute W_eff = W_nom / k.
    4. Report k_pos / k_neg only when gain asymmetry exceeds the threshold.
    5. Estimate the angular dead-band from dead_band rows.

    Parameters
    ----------
    rows : list[dict]
        Raw rows as produced by the sweep thread or loaded via ``_load_csv``.
    config : DriveCalConfig
        Calibration configuration (used for W_nom and calibration_surface).

    Returns
    -------
    dict
        Complete ``ugv_drive`` section (without ``calibrated_at`` and
        ``raw_csv`` — the caller fills those in).  Contains an ``"error"``
        key if analysis failed due to insufficient good samples.
    """
    quality_passed = [
        r
        for r in rows
        if r["run_type"] == _GAIN_RUN
        and int(r["quality_flag"]) == 0
        and r.get("corrected_delta_deg") is not None
        and r.get("expected_angle_deg") is not None
    ]
    n_sign_inconsistent = sum(
        1
        for r in quality_passed
        if not _sign_consistent(
            float(r["corrected_delta_deg"]), float(r["expected_angle_deg"])
        )
    )
    if n_sign_inconsistent > 0:
        logger.warning(
            f"analyse_runs: excluding {n_sign_inconsistent} sign-inconsistent "
            "gain sample(s) — rover rotated opposite to commanded direction."
        )
    gain_good = [
        r
        for r in quality_passed
        if _sign_consistent(
            float(r["corrected_delta_deg"]), float(r["expected_angle_deg"])
        )
    ]
    n_good = len(gain_good)
    n_total = len(rows)

    if n_good < 2:
        logger.error(
            f"analyse_runs: only {n_good} good gain_sweep sample(s) — "
            "need at least 2 to fit the turn-rate gain."
        )
        return {
            "n_samples": n_good,
            "n_total_samples": n_total,
            "error": "insufficient_good_samples",
        }

    corrected = [float(r["corrected_delta_deg"]) for r in gain_good]
    expected = [float(r["expected_angle_deg"]) for r in gain_good]
    fit_all = fit_turn_rate_gain(corrected, expected)
    k = fit_all["slope"]

    # Fit CCW and CW separately for asymmetry check.
    ccw_rows = [r for r in gain_good if r["direction"] == "ccw"]
    cw_rows = [r for r in gain_good if r["direction"] == "cw"]

    k_pos: float | None = None
    k_neg: float | None = None
    asymmetric = False

    if len(ccw_rows) >= 2 and len(cw_rows) >= 2:
        fit_ccw = fit_turn_rate_gain(
            [float(r["corrected_delta_deg"]) for r in ccw_rows],
            [float(r["expected_angle_deg"]) for r in ccw_rows],
        )
        fit_cw = fit_turn_rate_gain(
            [float(r["corrected_delta_deg"]) for r in cw_rows],
            [float(r["expected_angle_deg"]) for r in cw_rows],
        )
        k_pos = fit_ccw["slope"]  # positive: CCW delta / CCW expected
        k_neg = fit_cw["slope"]  # also positive: CW delta / CW expected (both negative)
        asymmetry = abs(k_pos - k_neg) / max(abs(k_pos), abs(k_neg))
        asymmetric = asymmetry > _ASYMMETRY_THRESHOLD
        if asymmetric:
            logger.warning(
                f"Motor asymmetry detected: k_pos={k_pos:.4f}, k_neg={k_neg:.4f} "
                f"(asymmetry={asymmetry:.3f} > threshold={_ASYMMETRY_THRESHOLD})"
            )

    w_eff = config.ugv_track_width_nom / k if k > 0 else None

    # Dead-band analysis.
    omega_dead = _estimate_dead_band_from_rows(rows)

    return {
        "calibration_surface": config.calibration_surface,
        "effective_track_width_m": round(w_eff, 6) if w_eff is not None else None,
        "turn_rate_gain": round(k, 6),
        "turn_rate_gain_pos": round(k_pos, 6)
        if asymmetric and k_pos is not None
        else None,
        "turn_rate_gain_neg": round(k_neg, 6)
        if asymmetric and k_neg is not None
        else None,
        "angular_dead_band_rad_s": round(omega_dead, 4)
        if omega_dead is not None
        else None,
        "n_samples": n_good,
        "fit_mae_deg": round(fit_all["mae_deg"], 4),
        "n_total_samples": n_total,
    }


def _estimate_dead_band_from_rows(rows: list[dict[str, Any]]) -> float | None:
    """Extract dead-band threshold from raw dead_band rows.

    Evaluates CCW and CW probes independently and returns the more
    conservative (larger) estimate.

    Parameters
    ----------
    rows : list[dict]
        All raw rows (gain_sweep rows are ignored).

    Returns
    -------
    float or None
        Conservative dead-band estimate in rad/s, or None if no dead-band
        data is present.
    """
    dead_rows = [r for r in rows if r["run_type"] == _DEAD_RUN]
    if not dead_rows:
        return None

    def _parse_moved(value: Any) -> bool | None:
        """Parse a stored ``moved`` value into bool.

        Accepts booleans, numeric 0/1, and string encodings such as
        ``"0"``, ``"1"``, ``"true"``, and ``"false"``.
        Returns ``None`` when the value cannot be interpreted.
        """
        if value is None:
            return None
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            if value == 0:
                return False
            if value == 1:
                return True
            return None
        if isinstance(value, str):
            v = value.strip().lower()
            if v in {"0", "false", "f", "no", "n"}:
                return False
            if v in {"1", "true", "t", "yes", "y"}:
                return True
        return None

    def _estimate_for_direction(direction: str) -> float | None:
        dir_rows = [
            r
            for r in dead_rows
            if r["direction"] == direction and r.get("moved") is not None
        ]
        if not dir_rows:
            return None
        # Sort descending by omega.
        dir_rows_sorted = sorted(
            dir_rows, key=lambda r: -float(r["omega_commanded_rad_s"])
        )
        parsed = [
            (float(r["omega_commanded_rad_s"]), _parse_moved(r.get("moved")))
            for r in dir_rows_sorted
        ]
        valid = [(omega, moved) for omega, moved in parsed if moved is not None]
        if not valid:
            return None
        omegas = [omega for omega, _ in valid]
        moved_flags = [moved for _, moved in valid]
        return estimate_dead_band(omegas, moved_flags)

    ccw_est = _estimate_for_direction("ccw")
    cw_est = _estimate_for_direction("cw")

    candidates = [x for x in [ccw_est, cw_est] if x is not None]
    if not candidates:
        return None
    return max(candidates)


# ---------------------------------------------------------------------------
# Config parsing
# ---------------------------------------------------------------------------


def _validate_geometry(camera_offset_m: float, target_distance_m: float) -> None:
    """Validate camera offset and target distance geometry parameters.

    Raises
    ------
    ValueError
        If any constraint is violated.
    """
    if target_distance_m <= 0:
        raise ValueError(
            f"ugv_drive.target_distance_m must be positive, got {target_distance_m}"
        )
    if camera_offset_m < 0:
        raise ValueError(
            f"ugv_drive.camera_offset_m must be non-negative, got {camera_offset_m}"
        )
    if camera_offset_m >= target_distance_m:
        raise ValueError(
            f"ugv_drive.camera_offset_m ({camera_offset_m}) must be less than "
            f"ugv_drive.target_distance_m ({target_distance_m})"
        )


def _load_config(
    sensor_cfg: dict[str, Any],
    cal_cfg: dict[str, Any],
    args: argparse.Namespace,
) -> DriveCalConfig:
    """Parse and validate both config dicts + CLI args into a ``DriveCalConfig``.

    Raises
    ------
    ValueError
        If required fields are missing, null, or out of range.
    KeyError
        If required top-level sections are absent from the sensor config.
    """
    # --- Fisheye intrinsics ---
    K, D = load_fisheye_intrinsics(sensor_cfg)
    ws_cfg = sensor_cfg.get("waveshare_rgb", {})
    res = ws_cfg.get("resolution", [1280, 720])
    frame_width, frame_height = int(res[0]), int(res[1])
    cx = float(K[0, 2])

    # --- UGV hardware ---
    ugv_cfg: dict[str, Any] = sensor_cfg["ugv"]

    # --- Calibration parameters ---
    ud_cfg: dict[str, Any] = cal_cfg.get("ugv_drive")  # type: ignore[assignment]
    if ud_cfg is None:
        raise ValueError(
            "ugv_drive section is missing from calibration_config.yaml. "
            "Add it before running calibration."
        )

    omega_raw = ud_cfg.get("omega_commands_rad_s")
    if not omega_raw:
        raise ValueError("ugv_drive.omega_commands_rad_s must be a non-empty list.")
    omega_cmds = tuple(float(v) for v in omega_raw)

    dead_raw = ud_cfg.get("dead_band_omega_steps_rad_s")
    if not dead_raw:
        raise ValueError(
            "ugv_drive.dead_band_omega_steps_rad_s must be a non-empty list."
        )
    dead_steps = tuple(float(v) for v in dead_raw)

    camera_offset = float(ud_cfg.get("camera_offset_m", 0.0))
    target_dist = float(ud_cfg.get("target_distance_m", 2.0))
    _validate_geometry(camera_offset, target_dist)

    noise_floor = float(ud_cfg.get("noise_floor_deg", 1.5))
    if args.noise_floor is not None:
        noise_floor = float(args.noise_floor)

    camera_device = args.camera_device or str(
        ud_cfg.get("camera_device", "/dev/video0")
    )

    return DriveCalConfig(
        K=K,
        D=D,
        frame_width=frame_width,
        frame_height=frame_height,
        cx=cx,
        ugv_port=str(ugv_cfg["port"]),
        ugv_baud_rate=int(ugv_cfg["baud_rate"]),
        ugv_chassis_main=int(ugv_cfg["chassis_main"]),
        ugv_chassis_module=int(ugv_cfg["chassis_module"]),
        ugv_track_width_nom=float(ugv_cfg["track_width"]),
        omega_commands_rad_s=omega_cmds,
        command_duration_s=float(ud_cfg.get("command_duration_s", 2.0)),
        settle_time_s=float(ud_cfg.get("settle_time_s", 2.0)),
        dead_band_omega_steps_rad_s=dead_steps,
        dead_band_duration_s=float(ud_cfg.get("dead_band_duration_s", 1.5)),
        dead_band_settle_s=float(ud_cfg.get("dead_band_settle_s", 1.5)),
        frames_to_average=int(ud_cfg.get("frames_to_average", 5)),
        noise_floor_deg=noise_floor,
        camera_offset_m=camera_offset,
        target_distance_m=target_dist,
        template_half_width_px=int(ud_cfg.get("template_half_width_px", 80)),
        min_match_score=float(ud_cfg.get("min_match_score", 0.65)),
        camera_device=camera_device,
        calibration_surface=str(ud_cfg.get("calibration_surface", "unspecified")),
        sensor_config_path=args.sensor_config.resolve(),
    )


# ---------------------------------------------------------------------------
# Template matching helpers
# ---------------------------------------------------------------------------


def _capture_template(
    cap: cv2.VideoCapture,
    cx: float,
    cy: float,
    half_w: int,
) -> cv2.typing.MatLike:
    """Capture one frame and extract a square template centred at (cx, cy).

    Parameters
    ----------
    cap : cv2.VideoCapture
        Opened camera capture device.
    cx : float
        Horizontal centre for the template crop (principal point x).
    cy : float
        Vertical centre for the template crop (principal point y).
    half_w : int
        Half-width of the square template in pixels.

    Returns
    -------
    cv2.typing.MatLike
        The cropped template patch.

    Raises
    ------
    RuntimeError
        If no frame can be read from the camera.
    """
    ok, frame = cap.read()
    if not ok or frame is None:
        raise RuntimeError("Failed to capture frame for template extraction.")
    h, w = frame.shape[:2]
    cx_i = int(round(cx))
    cy_i = int(round(cy))
    y1 = max(0, cy_i - half_w)
    y2 = min(h, cy_i + half_w)
    x1 = max(0, cx_i - half_w)
    x2 = min(w, cx_i + half_w)
    return frame[y1:y2, x1:x2].copy()


def _match_template_uv(
    frame: cv2.typing.MatLike,
    template: cv2.typing.MatLike,
) -> tuple[float, float, float]:
    """Run TM_CCOEFF_NORMED template matching and return centroid and score.

    Parameters
    ----------
    frame : cv2.typing.MatLike
        Full camera frame.
    template : cv2.typing.MatLike
        Template patch as captured by ``_capture_template``.

    Returns
    -------
    (centroid_u, centroid_v, match_score) : tuple[float, float, float]
        ``centroid_u`` is the horizontal centre of the best match location.
        ``centroid_v`` is the vertical centre.
        ``match_score`` is the normalised correlation coefficient (0–1).
    """
    result = cv2.matchTemplate(frame, template, cv2.TM_CCOEFF_NORMED)
    _, max_val, _, max_loc = cv2.minMaxLoc(result)
    centroid_u = max_loc[0] + template.shape[1] / 2.0
    centroid_v = max_loc[1] + template.shape[0] / 2.0
    return centroid_u, centroid_v, float(max_val)


def _capture_bearing_measurement(
    cap: cv2.VideoCapture,
    template: cv2.typing.MatLike,
    frames_to_average: int,
    K: np.ndarray,
    D: np.ndarray,
    cap_lock: threading.Lock,
) -> tuple[float, float, float]:
    """Capture frames, template-match, and return a robust bearing estimate.

    Acquires ``cap_lock`` exclusively for the duration of frame collection.

    Parameters
    ----------
    cap : cv2.VideoCapture
        Open camera capture device.
    template : cv2.typing.MatLike
        Template image as captured by ``_capture_template``.
    frames_to_average : int
        Number of frames to collect and median-average.
    K : np.ndarray
        3×3 fisheye camera matrix.
    D : np.ndarray
        Fisheye distortion coefficients shaped (4, 1).
    cap_lock : threading.Lock
        Shared lock serialising camera access with the frame-display thread.

    Returns
    -------
    (median_bearing_deg, median_u_px, median_match_score) : tuple
        Median values over the captured frames.

    Raises
    ------
    RuntimeError
        If no frames can be read from the camera.
    """
    bearings: list[float] = []
    u_list: list[float] = []
    scores: list[float] = []
    with cap_lock:
        for _ in range(frames_to_average):
            ok, frame = cap.read()
            if not ok or frame is None:
                logger.warning(
                    "cap.read() failed during bearing measurement — skipping frame."
                )
                continue
            u, v, score = _match_template_uv(frame, template)
            bearing = pixel_to_bearing_deg(u, v, K, D)
            bearings.append(bearing)
            u_list.append(u)
            scores.append(score)
    if not bearings:
        raise RuntimeError("No frames could be read during bearing measurement.")
    return (
        float(np.median(bearings)),
        float(np.median(u_list)),
        float(np.median(scores)),
    )


# ---------------------------------------------------------------------------
# CSV I/O
# ---------------------------------------------------------------------------


def _make_csv_path() -> Path:
    """Generate a timestamped CSV path under ``_CSV_DIR``."""
    ts = datetime.now().strftime("%Y%m%dT%H%M%S")
    return _CSV_DIR / f"{ts}.csv"


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    """Write rows to a CSV file using the canonical column order."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=_CSV_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    logger.info(f"CSV written to {path} ({len(rows)} rows).")


def _load_csv(path: Path) -> list[dict[str, Any]]:
    """Load a CSV written by ``_write_csv`` and return rows as typed dicts.

    Numeric fields are cast to ``float``; integer columns to ``int``;
    string columns remain ``str``; optional float columns become ``None``
    for empty strings; the ``moved`` column becomes a ``bool`` or ``None``.
    """
    rows: list[dict[str, Any]] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            typed: dict[str, Any] = {}
            for k, v in row.items():
                if k in _CSV_INT_COLS:
                    typed[k] = int(v)
                elif k in _CSV_STR_COLS:
                    typed[k] = v
                elif k in _CSV_OPTIONAL_FLOAT_COLS:
                    typed[k] = float(v) if v else None
                elif k in _CSV_BOOL_COLS:
                    typed[k] = (v == "1") if v else None
                else:
                    typed[k] = float(v)
            rows.append(typed)
    return rows


# ---------------------------------------------------------------------------
# Row builders
# ---------------------------------------------------------------------------


def _build_gain_row(
    omega: float,
    direction: str,
    duration_s: float,
    b_before: float,
    b_after: float,
    corrected_delta: float,
    expected_angle: float,
    score_before: float,
    score_after: float,
    qflag: int,
    t0: float,
) -> dict[str, Any]:
    """Build a gain_sweep CSV row dict."""
    return {
        "timestamp_s": round(time.monotonic() - t0, 4),
        "run_type": _GAIN_RUN,
        "omega_commanded_rad_s": round(omega, 6),
        "direction": direction,
        "command_duration_s": round(duration_s, 4),
        "bearing_before_deg": round(b_before, 6),
        "bearing_after_deg": round(b_after, 6),
        "delta_bearing_deg": round(b_after - b_before, 6),
        "corrected_delta_deg": round(corrected_delta, 6),
        "expected_angle_deg": round(expected_angle, 6),
        "moved": "",
        "match_score_before": round(score_before, 6),
        "match_score_after": round(score_after, 6),
        "quality_flag": qflag,
    }


def _build_dead_band_row(
    omega: float,
    direction: str,
    duration_s: float,
    b_before: float,
    b_after: float,
    moved: bool,
    score_before: float,
    score_after: float,
    qflag: int,
    t0: float,
) -> dict[str, Any]:
    """Build a dead_band CSV row dict."""
    return {
        "timestamp_s": round(time.monotonic() - t0, 4),
        "run_type": _DEAD_RUN,
        "omega_commanded_rad_s": round(omega, 6),
        "direction": direction,
        "command_duration_s": round(duration_s, 4),
        "bearing_before_deg": round(b_before, 6),
        "bearing_after_deg": round(b_after, 6),
        "delta_bearing_deg": round(b_after - b_before, 6),
        "corrected_delta_deg": "",
        "expected_angle_deg": "",
        "moved": "1" if moved else "0",
        "match_score_before": round(score_before, 6),
        "match_score_after": round(score_after, 6),
        "quality_flag": qflag,
    }


# ---------------------------------------------------------------------------
# Sweep thread — cancellation sentinel and helpers
# ---------------------------------------------------------------------------


class _SweepCancelled(Exception):
    """Raised internally to short-circuit a cancelled sweep without triggering FAILED."""


def _check_cancel(cancel_event: threading.Event) -> None:
    """Raise ``_SweepCancelled`` if *cancel_event* is set.

    Parameters
    ----------
    cancel_event : threading.Event
        Shared cancellation signal from the orchestrator.

    Raises
    ------
    _SweepCancelled
        When the event is set.
    """
    if cancel_event.is_set():
        raise _SweepCancelled()


def _execute_rotation(
    omega_signed: float,
    duration_s: float,
    settle_s: float,
    cap: cv2.VideoCapture,
    template: cv2.typing.MatLike,
    config: DriveCalConfig,
    ugv: UGVController,
    cap_lock: threading.Lock,
    cancel_event: threading.Event,
) -> tuple[float, float, float, float, float, float]:
    """Measure bearing, rotate, settle, and measure again.

    Parameters
    ----------
    omega_signed : float
        Angular velocity to command (rad/s), signed (negative = CW).
    duration_s : float
        How long to drive the rotation command (seconds).
    settle_s : float
        How long to wait after stopping before the post-rotation measurement.
    cap : cv2.VideoCapture
        Video capture device.
    template : cv2.typing.MatLike
        Template image for bearing measurement.
    config : DriveCalConfig
        Calibration configuration (intrinsics, frame dimensions, etc.).
    ugv : UGVController
        UGV hardware interface.
    cap_lock : threading.Lock
        Lock protecting *cap* from concurrent access.
    cancel_event : threading.Event
        Shared cancellation signal.

    Returns
    -------
    tuple[float, float, float, float, float, float]
        ``(b_before, u_before, score_before, b_after, u_after, score_after)``

    Raises
    ------
    _SweepCancelled
        If *cancel_event* is set after stopping or after settling.
    """
    b_before, u_before, score_before = _capture_bearing_measurement(
        cap, template, config.frames_to_average, config.K, config.D, cap_lock
    )
    ugv.move(0.0, omega_signed)
    time.sleep(duration_s)
    ugv.stop()
    _check_cancel(cancel_event)
    time.sleep(settle_s)
    _check_cancel(cancel_event)
    b_after, u_after, score_after = _capture_bearing_measurement(
        cap, template, config.frames_to_average, config.K, config.D, cap_lock
    )
    return b_before, u_before, score_before, b_after, u_after, score_after


def _run_gain_step(
    omega: float,
    direction: str,
    cap: cv2.VideoCapture,
    ugv: UGVController,
    config: DriveCalConfig,
    template: cv2.typing.MatLike,
    cap_lock: threading.Lock,
    cancel_event: threading.Event,
    t0: float,
) -> dict[str, Any]:
    """Execute one CCW or CW gain measurement run and return a row dict.

    Parameters
    ----------
    omega : float
        Unsigned angular velocity (rad/s); negated internally for CW.
    direction : str
        ``"ccw"`` or ``"cw"``.
    cap : cv2.VideoCapture
        Video capture device.
    ugv : UGVController
        UGV hardware interface.
    config : DriveCalConfig
        Calibration configuration.
    template : cv2.typing.MatLike
        Template image for bearing measurement.
    cap_lock : threading.Lock
        Lock protecting *cap* from concurrent access.
    cancel_event : threading.Event
        Shared cancellation signal.
    t0 : float
        ``time.monotonic()`` reference at sweep start, used for timestamps.

    Returns
    -------
    dict[str, Any]
        Gain sweep CSV row (see ``_build_gain_row``).

    Raises
    ------
    _SweepCancelled
        If the cancel event fires before or during the rotation.
    """
    _check_cancel(cancel_event)
    omega_signed = omega if direction == "ccw" else -omega
    b_before, u_before, score_before, b_after, u_after, score_after = _execute_rotation(
        omega_signed,
        config.command_duration_s,
        config.settle_time_s,
        cap,
        template,
        config,
        ugv,
        cap_lock,
        cancel_event,
    )
    delta = b_after - b_before
    corrected = correct_for_camera_offset(
        delta, config.camera_offset_m, config.target_distance_m
    )
    expected = math.degrees(omega_signed * config.command_duration_s)
    qflag = quality_flag(
        score_before, u_before, config.frame_width, config.min_match_score
    ) | quality_flag(score_after, u_after, config.frame_width, config.min_match_score)
    if abs(corrected) < config.noise_floor_deg or abs(corrected) < _MIN_ROTATION_FRACTION * abs(expected):
        qflag |= _STALL_FLAG
    label = "CCW" if direction == "ccw" else "CW "
    logger.info(
        f"Gain {label} {omega:.2f} rad/s: Δ={delta:+.2f}°  "
        f"corrected={corrected:+.2f}°  expected={expected:+.2f}°  flag={qflag}"
    )
    return _build_gain_row(
        omega=omega,
        direction=direction,
        duration_s=config.command_duration_s,
        b_before=b_before,
        b_after=b_after,
        corrected_delta=corrected,
        expected_angle=expected,
        score_before=score_before,
        score_after=score_after,
        qflag=qflag,
        t0=t0,
    )


def _run_dead_band_step(
    omega_d: float,
    direction: str,
    cap: cv2.VideoCapture,
    ugv: UGVController,
    config: DriveCalConfig,
    template: cv2.typing.MatLike,
    cap_lock: threading.Lock,
    cancel_event: threading.Event,
    t0: float,
) -> dict[str, Any]:
    """Execute one CCW or CW dead-band probe and return a row dict.

    Parameters
    ----------
    omega_d : float
        Unsigned angular velocity step (rad/s); negated internally for CW.
    direction : str
        ``"ccw"`` or ``"cw"``.
    cap : cv2.VideoCapture
        Video capture device.
    ugv : UGVController
        UGV hardware interface.
    config : DriveCalConfig
        Calibration configuration.
    template : cv2.typing.MatLike
        Template image for bearing measurement.
    cap_lock : threading.Lock
        Lock protecting *cap* from concurrent access.
    cancel_event : threading.Event
        Shared cancellation signal.
    t0 : float
        ``time.monotonic()`` reference at sweep start, used for timestamps.

    Returns
    -------
    dict[str, Any]
        Dead-band CSV row (see ``_build_dead_band_row``).

    Raises
    ------
    _SweepCancelled
        If the cancel event fires before or during the probe.
    """
    _check_cancel(cancel_event)
    omega_signed = omega_d if direction == "ccw" else -omega_d
    b_before, u_before, score_before, b_after, u_after, score_after = _execute_rotation(
        omega_signed,
        config.dead_band_duration_s,
        config.dead_band_settle_s,
        cap,
        template,
        config,
        ugv,
        cap_lock,
        cancel_event,
    )
    delta = b_after - b_before
    did_move = abs(delta) > config.noise_floor_deg
    qflag = quality_flag(
        score_before, u_before, config.frame_width, config.min_match_score
    ) | quality_flag(score_after, u_after, config.frame_width, config.min_match_score)
    label = "CCW" if direction == "ccw" else "CW "
    logger.info(
        f"Dead-band {label} {omega_d:.2f} rad/s: Δ={delta:+.2f}°  moved={did_move}"
    )
    return _build_dead_band_row(
        omega=omega_d,
        direction=direction,
        duration_s=config.dead_band_duration_s,
        b_before=b_before,
        b_after=b_after,
        moved=did_move,
        score_before=score_before,
        score_after=score_after,
        qflag=qflag,
        t0=t0,
    )


def _wait_for_recenter(
    state: CalibrationStateContainer,
    current_step: int,
    total_steps: int,
    recenter_event: threading.Event,
    cancel_event: threading.Event,
    next_block_label: str,
) -> None:
    """Pause the sweep and block until the operator confirms re-centring.

    Clears ``recenter_event`` before transitioning to ``WAITING_RECENTER`` so
    that a stale set from a previous pause cannot bypass the wait.

    Parameters
    ----------
    state : CalibrationStateContainer
        Shared state container; transitions to ``WAITING_RECENTER`` then back
        to ``SWEEPING`` when resumed.
    current_step : int
        Completed step count, used for progress reporting.
    total_steps : int
        Total step count across all blocks.
    recenter_event : threading.Event
        Set by the operator via ``GET /confirm`` to resume after re-centring.
        Also set by ``cancel_sweep()`` so that ``/abort`` unblocks the wait.
    cancel_event : threading.Event
        Checked periodically during the wait; raises ``_SweepCancelled`` if set.
    next_block_label : str
        Human-readable label for the block that follows this pause, used in
        log messages so the operator knows what is coming next.

    Raises
    ------
    _SweepCancelled
        If the cancel event fires while waiting for re-centre confirmation.
    """
    recenter_event.clear()
    state.transition_to(
        SweepStatus(
            state=CalibrationState.WAITING_RECENTER,
            progress_pct=100.0 * current_step / total_steps,
            current_step=current_step,
            total_steps=total_steps,
        )
    )
    logger.info(
        "Block complete ({}/{}). Waiting for operator to re-centre rover "
        "and GET /confirm before {}.",
        current_step,
        total_steps,
        next_block_label,
    )
    while not recenter_event.wait(timeout=0.5):
        _check_cancel(cancel_event)
    _check_cancel(cancel_event)
    logger.info("Recenter confirmed — starting {}.", next_block_label)


def _run_sweep(
    cap: cv2.VideoCapture,
    ugv: UGVController,
    config: DriveCalConfig,
    template: cv2.typing.MatLike,
    state: CalibrationStateContainer,
    cap_lock: threading.Lock,
    cancel_event: threading.Event,
    recenter_event: threading.Event,
    t0: float,
) -> None:
    """Execute the calibration sweep, then transition to COMPLETE or FAILED.

    Intended to run in a daemon thread.  For each commanded step:

    - Measure the target bearing before the rotation.
    - Command the UGV to rotate for ``command_duration_s`` seconds.
    - Wait ``settle_time_s`` after stopping.
    - Measure the bearing again and record the difference.

    The sweep runs four directional blocks in order:

    1. gain sweep CCW
    2. gain sweep CW
    3. dead-band sweep CCW
    4. dead-band sweep CW

    After each completed block except the last the state transitions to
    ``WAITING_RECENTER`` and the thread blocks until the operator calls
    ``GET /confirm`` (which sets ``recenter_event``).  This lets the
    operator manually re-centre the rover on the calibration target between
    every directional block.

    Dead-band sweep: decreasing ω values; records whether rotation occurred.

    On completion: calls ``state.set_sweep_rows`` then transitions to COMPLETE.
    On cancellation: returns without transitioning (``/abort`` handles ABORTED).
    On exception: transitions to FAILED.

    Parameters
    ----------
    recenter_event : threading.Event
        Set by the operator via ``GET /confirm`` to resume after re-centring.
        Also set by ``cancel_sweep()`` so that ``/abort`` unblocks the wait.
    t0 : float
        ``time.monotonic()`` reference at sweep start, used for timestamps.
    """
    total_steps = (
        len(config.omega_commands_rad_s) * 2
        + len(config.dead_band_omega_steps_rad_s) * 2
    )
    current_step = 0
    rows: list[dict[str, Any]] = []
    cy = config.frame_height / 2.0

    # Ordered directional blocks: (step_fn, direction, omegas, label).
    # Block boundaries drive the recenter pause locations dynamically.
    blocks: list[
        tuple[Callable[..., dict[str, Any]], str, Sequence[float], str]
    ] = [
        (_run_gain_step,      "ccw", config.omega_commands_rad_s,        "gain sweep CCW"),
        (_run_gain_step,      "cw",  config.omega_commands_rad_s,        "gain sweep CW"),
        (_run_dead_band_step, "ccw", config.dead_band_omega_steps_rad_s, "dead-band sweep CCW"),
        (_run_dead_band_step, "cw",  config.dead_band_omega_steps_rad_s, "dead-band sweep CW"),
    ]

    try:
        for block_idx, (step_fn, direction, omegas, label) in enumerate(blocks):
            logger.info("Starting {}.", label)
            mid = len(omegas) // 2
            for step_idx, omega in enumerate(omegas):
                current_step += 1
                rows.append(
                    step_fn(
                        omega,
                        direction,
                        cap,
                        ugv,
                        config,
                        template,
                        cap_lock,
                        cancel_event,
                        t0,
                    )
                )
                state.update_sweep_progress(current_step, total_steps)

                if step_idx + 1 == mid and mid < len(omegas):
                    _wait_for_recenter(
                        state,
                        current_step,
                        total_steps,
                        recenter_event,
                        cancel_event,
                        label,
                    )
                    with cap_lock:
                        template = _capture_template(
                            cap, config.cx, cy, config.template_half_width_px
                        )
                    logger.info("Template recaptured after mid-block recenter.")

            if block_idx < len(blocks) - 1:
                _wait_for_recenter(
                    state,
                    current_step,
                    total_steps,
                    recenter_event,
                    cancel_event,
                    blocks[block_idx + 1][3],
                )
                with cap_lock:
                    template = _capture_template(
                        cap, config.cx, cy, config.template_half_width_px
                    )
                logger.info("Template recaptured after recenter.")

        state.set_sweep_rows(rows)
        state.transition_to(
            SweepStatus(
                state=CalibrationState.COMPLETE,
                progress_pct=100.0,
                current_step=total_steps,
                total_steps=total_steps,
            )
        )
        logger.info(f"Sweep complete — {len(rows)} measurement rows collected.")

    except _SweepCancelled:
        return  # /abort handles ABORTED transition
    except Exception as exc:
        logger.exception(f"Sweep thread raised an exception: {exc}")
        state.transition_to(
            SweepStatus(state=CalibrationState.FAILED, error_message=str(exc))
        )


# ---------------------------------------------------------------------------
# YAML write (atomic)
# ---------------------------------------------------------------------------


def _write_results_atomic(
    sensor_config_path: Path,
    ugv_drive_dict: dict[str, Any],
) -> None:
    """Merge ``ugv_drive`` section into sensor_config.yaml atomically.

    Reads the existing file, sets (or replaces) the top-level ``ugv_drive``
    key, and re-serialises via ``tempfile`` + ``os.replace``.  All other
    top-level keys and their values are preserved.

    Parameters
    ----------
    sensor_config_path : Path
        Path to ``sensor_config.yaml``.
    ugv_drive_dict : dict
        Calibration results to write under the ``ugv_drive`` key.
    """
    with sensor_config_path.open() as f:
        config: dict[str, Any] = yaml.safe_load(f) or {}

    config["ugv_drive"] = ugv_drive_dict

    tmp_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            dir=sensor_config_path.parent,
            suffix=".tmp",
            delete=False,
        ) as tmp:
            yaml.dump(config, tmp, default_flow_style=False, sort_keys=False)
            tmp_path = Path(tmp.name)
        os.replace(tmp_path, sensor_config_path)
        tmp_path = None
    finally:
        if tmp_path is not None:
            tmp_path.unlink(missing_ok=True)

    logger.info(f"Results written to {sensor_config_path}")


# ---------------------------------------------------------------------------
# Calibration orchestrator
# ---------------------------------------------------------------------------


class CalibrationOrchestrator:
    """Owns calibration logic: starting sweeps and persisting results.

    Decoupled from HTTP transport.

    Parameters
    ----------
    config : DriveCalConfig
    ugv    : UGVController
    cap    : cv2.VideoCapture
    state  : CalibrationStateContainer
    cap_lock : threading.Lock
        Shared lock serialising camera access between this orchestrator's
        sweep thread and the frame display thread.
    """

    def __init__(
        self,
        config: DriveCalConfig,
        ugv: UGVController,
        cap: cv2.VideoCapture,
        state: CalibrationStateContainer,
        cap_lock: threading.Lock,
    ) -> None:
        self._config = config
        self._ugv = ugv
        self._cap = cap
        self._state = state
        self._cap_lock = cap_lock
        self._cancel_event: threading.Event = threading.Event()
        self._recenter_event: threading.Event = threading.Event()
        self._csv_path: Path | None = None
        self._saved_result: dict[str, Any] | None = None

    def confirm_target(self) -> None:
        """Transition WAITING_TARGET → SWEEPING, capture template, spawn sweep thread.

        Raises
        ------
        RuntimeError
            If not currently in WAITING_TARGET state.
        """
        if not self._state.try_start_sweep():
            raise RuntimeError("not in WAITING_TARGET state")

        logger.info("Capturing template at current camera position...")
        h = self._config.frame_height
        cy = float(h) / 2.0
        with self._cap_lock:
            template = _capture_template(
                self._cap,
                self._config.cx,
                cy,
                self._config.template_half_width_px,
            )

        self._cancel_event.clear()
        self._recenter_event.clear()
        self._csv_path = _make_csv_path()
        t0 = time.monotonic()

        threading.Thread(
            target=_run_sweep,
            args=(
                self._cap,
                self._ugv,
                self._config,
                template,
                self._state,
                self._cap_lock,
                self._cancel_event,
                self._recenter_event,
                t0,
            ),
            daemon=True,
        ).start()
        logger.info(f"Sweep thread started.  CSV target: {self._csv_path}")

    def cancel_sweep(self) -> None:
        """Signal the running sweep thread (if any) to stop early."""
        self._cancel_event.set()
        self._recenter_event.set()  # unblock sweep thread if paused for recenter

    def resume_recenter(self) -> None:
        """Transition WAITING_RECENTER → SWEEPING and unblock the paused sweep thread.

        Raises
        ------
        RuntimeError
            If not currently in WAITING_RECENTER state.
        """
        if not self._state.try_start_recenter_resume():
            raise RuntimeError("not in WAITING_RECENTER state")
        self._recenter_event.set()
        logger.info("Operator confirmed recenter — continuing to dead-band sweep.")

    def save_results(self) -> tuple[Path, dict[str, Any]]:
        """Write CSV and sensor_config.yaml from the completed sweep.

        Idempotent: if already saved, returns the cached result.

        Returns
        -------
        (csv_path, model)

        Raises
        ------
        RuntimeError
            If not currently in COMPLETE state.
        """
        if self._saved_result is not None and self._csv_path is not None:
            logger.info("save_results() called again — returning cached result.")
            return self._csv_path, self._saved_result

        snapshot = self._state.get_snapshot()
        if snapshot.state != CalibrationState.COMPLETE:
            raise RuntimeError("not in COMPLETE state")

        rows = self._state.get_sweep_rows()
        if rows is None:
            raise RuntimeError("COMPLETE state but no sweep rows — this is a bug")

        assert self._csv_path is not None
        _write_csv(self._csv_path, rows)

        model = analyse_runs(rows, self._config)
        model["calibrated_at"] = datetime.now().isoformat(timespec="seconds")
        try:
            model["raw_csv"] = str(self._csv_path.relative_to(get_project_root()))
        except ValueError:
            model["raw_csv"] = str(self._csv_path)

        _write_results_atomic(self._config.sensor_config_path, model)
        self._saved_result = model
        return self._csv_path, model


# ---------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------


def _make_handler(
    state: CalibrationStateContainer,
    server_ref: list[ThreadingHTTPServer | None],
    orchestrator: CalibrationOrchestrator,
) -> type[BaseHTTPRequestHandler]:
    """Return a handler class closed over the shared state and orchestrator."""

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _fmt: str, *_args: object) -> None:
            pass

        def do_GET(self) -> None:  # noqa: N802
            if self.path == "/stream":
                self._serve_stream()
            elif self.path == "/status":
                self._serve_json(state.get_status())
            elif self.path == "/confirm":
                self._handle_confirm()
            elif self.path == "/abort":
                self._handle_abort()
            elif self.path == "/save":
                self._handle_save()
            elif self.path == "/reset":
                self._handle_reset()
            else:
                self.send_error(404)

        def _serve_json(self, data: dict[str, Any], status: int = 200) -> None:
            body = json.dumps(data).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _serve_stream(self) -> None:
            self.send_response(200)
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame",
            )
            self.end_headers()
            try:
                while True:
                    jpeg = state.get_annotated_jpeg()
                    if jpeg is None:
                        time.sleep(0.05)
                        continue
                    part = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                        + jpeg
                        + b"\r\n"
                    )
                    self.wfile.write(part)
                    self.wfile.flush()
                    time.sleep(1 / 30)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _handle_confirm(self) -> None:
            # /confirm handles both initial start (WAITING_TARGET) and
            # mid-sweep recenter resume (WAITING_RECENTER).
            current = state.get_status()["state"]
            try:
                if current == CalibrationState.WAITING_RECENTER.value:
                    orchestrator.resume_recenter()
                else:
                    orchestrator.confirm_target()
            except RuntimeError as exc:
                self._serve_json({"error": str(exc)}, 409)
                return
            self._serve_json({"status": "sweeping"})

        def _handle_abort(self) -> None:
            orchestrator.cancel_sweep()
            state.transition_to(SweepStatus(state=CalibrationState.ABORTED))
            srv = server_ref[0]
            if srv is not None:
                threading.Thread(target=srv.shutdown, daemon=True).start()
            self._serve_json({"status": "aborted"})

        def _handle_save(self) -> None:
            try:
                csv_path, model = orchestrator.save_results()
            except RuntimeError:
                self._serve_json({"error": "not in COMPLETE state"}, 409)
                return
            project_root = get_project_root()
            try:
                rel_path = str(csv_path.relative_to(project_root))
            except ValueError:
                rel_path = str(csv_path)
            self._serve_json(
                {
                    "saved": True,
                    "csv_path": rel_path,
                    "n_samples": model.get("n_samples"),
                    "n_total_samples": model.get("n_total_samples"),
                    "turn_rate_gain": model.get("turn_rate_gain"),
                    "effective_track_width_m": model.get("effective_track_width_m"),
                    "angular_dead_band_rad_s": model.get("angular_dead_band_rad_s"),
                    "fit_mae_deg": model.get("fit_mae_deg"),
                }
            )

        def _handle_reset(self) -> None:
            if not state.try_reset():
                self._serve_json({"error": "cannot reset while sweep is in progress"}, 409)
                return
            self._serve_json({"status": "reset"})

    return Handler


# ---------------------------------------------------------------------------
# Frame thread
# ---------------------------------------------------------------------------


def _get_status_text(
    current_state: str,
    status: dict[str, Any],
) -> tuple[str, tuple[int, int, int], float] | None:
    """Return overlay text, BGR colour, and font scale for the current state."""
    if current_state == CalibrationState.WAITING_TARGET.value:
        return "Centre target on crosshair, then GET /confirm", (0, 255, 0), 0.7
    if current_state == CalibrationState.SWEEPING.value:
        step = status["current_step"]
        total = status["total_steps"]
        pct = status["progress_pct"]
        return f"Sweeping: {step}/{total}  ({pct:.0f}%)", (0, 200, 255), 0.8
    if current_state == CalibrationState.WAITING_RECENTER.value:
        step = status["current_step"]
        total = status["total_steps"]
        return (
            f"Re-centre rover on target ({step}/{total}), then GET /confirm",
            (0, 255, 255),  # yellow
            0.7,
        )
    if current_state == CalibrationState.COMPLETE.value:
        return "COMPLETE — GET /save to persist results", (0, 255, 128), 0.8
    if current_state == CalibrationState.FAILED.value:
        err = status.get("error_message") or "FAILED"
        return f"FAILED: {err[:60]}", (0, 0, 255), 0.7
    return None


def _run_frame(
    cap: cv2.VideoCapture,
    cx: float,
    template_half_width_px: int,
    state: CalibrationStateContainer,
    cap_lock: threading.Lock,
    stop_event: threading.Event,
) -> None:
    """Read frames, draw the crosshair guide overlay, and push to shared state.

    Uses non-blocking lock acquisition so the sweep thread can hold the camera
    lock exclusively during frame collection without starving this thread.
    """
    cx_col = int(round(cx))
    last_good_frame: cv2.typing.MatLike | None = None
    while not stop_event.is_set():
        acquired = cap_lock.acquire(blocking=False)
        if not acquired:
            time.sleep(0.01)
            continue
        try:
            ok, frame = cap.read()
        finally:
            cap_lock.release()

        if not ok or frame is None:
            if last_good_frame is None:
                time.sleep(0.05)
                continue
            frame = last_good_frame
        else:
            last_good_frame = frame

        h, w = frame.shape[:2]
        cy_row = int(round(h / 2.0))
        annotated = frame.copy()
        cv2.line(annotated, (cx_col, 0), (cx_col, h - 1), (0, 255, 0), 2)

        x1 = max(0, cx_col - template_half_width_px)
        x2 = min(w - 1, cx_col + template_half_width_px)
        y1 = max(0, cy_row - template_half_width_px)
        y2 = min(h - 1, cy_row + template_half_width_px)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.circle(annotated, (cx_col, cy_row), 4, (0, 255, 255), -1)

        status = state.get_status()
        overlay = _get_status_text(status["state"], status)
        if overlay is not None:
            text, color, scale = overlay
            cv2.putText(
                annotated,
                text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                scale,
                color,
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            annotated,
            f"cx = {cx:.1f} px",
            (10, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
        state.update_frame(annotated)


# ---------------------------------------------------------------------------
# Replay mode
# ---------------------------------------------------------------------------


def _run_replay(
    csv_path: Path,
    sensor_config_path: Path,
    config: DriveCalConfig,
    save: bool,
) -> None:
    """Load an existing CSV, refit all models, print a summary, optionally save.

    Parameters
    ----------
    csv_path            : Path  Path to the raw CSV file.
    sensor_config_path  : Path  Destination for sensor_config.yaml if ``save=True``.
    config              : DriveCalConfig  Provides W_nom, surface, geometry params.
    save                : bool  If True, write results to sensor_config.yaml.
    """
    if not csv_path.exists():
        logger.error(f"CSV not found: {csv_path}")
        sys.exit(1)

    rows = _load_csv(csv_path)
    logger.info(f"Loaded {len(rows)} rows from {csv_path}")

    model = analyse_runs(rows, config)

    print("\n=== Rover Drive Calibration — Replay Summary ===")
    print(f"Total rows      : {model.get('n_total_samples')}")
    print(f"Good gain samples: {model.get('n_samples')}")

    if model.get("error"):
        print(f"Analysis failed : {model['error']}")
        print("=" * 49)
        if save:
            logger.error("Replay analysis failed; refusing to save invalid results.")
            sys.exit(1)
        return

    print(f"Turn-rate gain  : {model.get('turn_rate_gain')}")
    if model.get("turn_rate_gain_pos") is not None:
        print(f"  k_pos (CCW)   : {model.get('turn_rate_gain_pos')}")
        print(f"  k_neg (CW)    : {model.get('turn_rate_gain_neg')}")
    print(f"W_eff           : {model.get('effective_track_width_m')} m")
    print(f"Dead-band       : {model.get('angular_dead_band_rad_s')} rad/s")
    print(f"Fit MAE         : {model.get('fit_mae_deg')} °")
    print(f"Surface         : {model.get('calibration_surface')}")
    print("=" * 49)

    if save:
        if not sensor_config_path.exists():
            logger.error(f"Sensor config not found: {sensor_config_path}")
            sys.exit(1)
        model["calibrated_at"] = datetime.now().isoformat(timespec="seconds")
        try:
            model["raw_csv"] = str(csv_path.relative_to(get_project_root()))
        except ValueError:
            model["raw_csv"] = str(csv_path)
        _write_results_atomic(sensor_config_path, model)
        logger.info(f"Results saved to {sensor_config_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _build_arg_parser() -> argparse.ArgumentParser:
    """Build and return the CLI argument parser."""
    parser = argparse.ArgumentParser(
        description=(
            "Characterise rover turn-rate gain and angular dead-band using "
            "the Waveshare fisheye camera as the angle-measurement instrument."
        )
    )
    parser.add_argument(
        "--replay",
        type=Path,
        default=None,
        metavar="CSV",
        help="Path to an existing raw CSV for offline refit (no hardware required).",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="When used with --replay, write results to sensor_config.yaml.",
    )
    parser.add_argument(
        "--camera-device",
        type=str,
        default=None,
        metavar="PATH",
        help=(
            "V4L2 device path for the Waveshare RGB camera "
            "(overrides calibration_config.yaml; default: /dev/video0)."
        ),
    )
    parser.add_argument(
        "--sensor-config",
        type=Path,
        default=_DEFAULT_SENSOR_CONFIG,
        metavar="PATH",
        help=f"Path to sensor_config.yaml (default: {_DEFAULT_SENSOR_CONFIG}).",
    )
    parser.add_argument(
        "--cal-config",
        type=Path,
        default=_DEFAULT_CAL_CONFIG,
        metavar="PATH",
        help=f"Path to calibration_config.yaml (default: {_DEFAULT_CAL_CONFIG}).",
    )
    parser.add_argument(
        "--noise-floor",
        type=float,
        default=None,
        metavar="DEG",
        help="Override ugv_drive.noise_floor_deg from calibration config.",
    )
    return parser


def _run_replay_mode(args: argparse.Namespace, sensor_config_path: Path) -> None:
    """Load config and run offline replay analysis.

    Parameters
    ----------
    args               : argparse.Namespace  Parsed CLI arguments.
    sensor_config_path : Path               Resolved path to sensor_config.yaml.
    """
    cal_config_path: Path = args.cal_config.resolve()
    if not cal_config_path.exists():
        logger.error(f"Calibration config not found: {cal_config_path}")
        sys.exit(1)
    with sensor_config_path.open() as f:
        sensor_cfg: dict[str, Any] = yaml.safe_load(f) or {}
    with cal_config_path.open() as f:
        cal_cfg: dict[str, Any] = yaml.safe_load(f) or {}
    try:
        config = _load_config(sensor_cfg, cal_cfg, args)
    except (ValueError, KeyError) as exc:
        logger.error(str(exc))
        sys.exit(1)
    _run_replay(
        csv_path=args.replay.resolve(),
        sensor_config_path=sensor_config_path,
        config=config,
        save=args.save,
    )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    args = _build_arg_parser().parse_args()

    sensor_config_path: Path = args.sensor_config.resolve()
    if not sensor_config_path.exists():
        logger.error(f"Sensor config not found: {sensor_config_path}")
        sys.exit(1)

    # ------------------------------------------------------------------
    # Replay (offline) mode — no hardware required
    # ------------------------------------------------------------------
    if args.replay is not None:
        _run_replay_mode(args, sensor_config_path)
        return

    # ------------------------------------------------------------------
    # Live mode — hardware required
    # ------------------------------------------------------------------
    cal_config_path = args.cal_config.resolve()
    if not cal_config_path.exists():
        logger.error(f"Calibration config not found: {cal_config_path}")
        sys.exit(1)

    with sensor_config_path.open() as f:
        sensor_cfg: dict[str, Any] = yaml.safe_load(f) or {}
    with cal_config_path.open() as f:
        cal_cfg: dict[str, Any] = yaml.safe_load(f) or {}

    try:
        config = _load_config(sensor_cfg, cal_cfg, args)
    except (ValueError, KeyError) as exc:
        logger.error(str(exc))
        sys.exit(1)

    logger.info(
        f"Fisheye intrinsics: cx={config.cx:.2f} px  "
        f"K[0,0]={config.K[0, 0]:.2f} px  resolution={config.frame_width}×{config.frame_height}"
    )
    logger.info(
        f"Gain sweep: {config.omega_commands_rad_s} rad/s  "
        f"duration={config.command_duration_s}s  settle={config.settle_time_s}s"
    )
    logger.info(
        f"Dead-band: {config.dead_band_omega_steps_rad_s} rad/s  "
        f"duration={config.dead_band_duration_s}s  settle={config.dead_band_settle_s}s"
    )
    logger.info(
        f"Camera offset: {config.camera_offset_m} m  "
        f"target dist: {config.target_distance_m} m  "
        f"noise floor: {config.noise_floor_deg}°"
    )

    ugv = UGVController(
        port=config.ugv_port,
        baud_rate=config.ugv_baud_rate,
        chassis_main=config.ugv_chassis_main,
        chassis_module=config.ugv_chassis_module,
        track_width=config.ugv_track_width_nom,
    )

    cap: cv2.VideoCapture | None = None
    frame_thread: threading.Thread | None = None
    server: ThreadingHTTPServer | None = None
    server_ref: list[ThreadingHTTPServer | None] = [None]
    state = CalibrationStateContainer()
    stop_event = threading.Event()
    cap_lock = threading.Lock()

    try:
        ugv.connect()
        time.sleep(_CONNECT_SETTLE_S)

        try:
            ensure_camera_device_available(config.camera_device)
        except RuntimeError as exc:
            logger.error(str(exc))
            sys.exit(1)

        cap = cv2.VideoCapture(config.camera_device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.frame_height)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
        if not cap.isOpened():
            logger.error(
                f"Could not open camera device {config.camera_device}. "
                "Try a different --camera-device (e.g. /dev/video0)."
            )
            sys.exit(1)

        frame_thread = threading.Thread(
            target=_run_frame,
            args=(
                cap,
                config.cx,
                config.template_half_width_px,
                state,
                cap_lock,
                stop_event,
            ),
            daemon=True,
        )
        frame_thread.start()
        logger.info("Frame thread started.")

        orchestrator = CalibrationOrchestrator(config, ugv, cap, state, cap_lock)
        server = ThreadingHTTPServer(
            ("0.0.0.0", _STREAM_PORT),
            _make_handler(state, server_ref, orchestrator),
        )
        server_ref[0] = server

        logger.info(
            f"HTTP server running on port {_STREAM_PORT}. "
            f"Open http://<pi-ip>:{_STREAM_PORT}/stream in your browser."
        )
        logger.info(
            "Endpoints: /stream (MJPEG), /status (JSON), "
            "/confirm (start sweep), /abort, /save, /reset"
        )

        try:
            server.serve_forever()
        except KeyboardInterrupt:
            logger.info("Shutting down...")

    finally:
        stop_event.set()
        if frame_thread is not None:
            frame_thread.join(timeout=2)
            if frame_thread.is_alive():
                logger.warning("Frame thread did not stop within 2 s.")
        if cap is not None:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
            cap.release()
        if server is not None:
            server.server_close()
        ugv.disconnect()
