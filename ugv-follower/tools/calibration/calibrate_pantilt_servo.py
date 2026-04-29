"""Pan-tilt servo curve calibration for the Waveshare pan-tilt module.

Measures the command-to-angle mapping of the pan axis by sweeping commanded pan
setpoints across a configurable range, measuring the settled pan angle from the
Waveshare RGB camera via template matching, and fitting a piecewise-linear model
to the result.

Angle measurement model
-----------------------
All pan-angle measurements use the **offset-corrected forward-displacement model**.
The correction accounts for the camera lens being displaced forward of the
pan-tilt rotation centre by ``camera_forward_offset_m`` metres::
    phi_corrected = phi_cam − arcsin( d/D · sin(phi_cam) )
where ``phi_cam`` is the naive arctan angle from the image centroid, ``d`` is
``camera_forward_offset_m``, and ``D`` is ``calibration_target_distance_m``.
Both geometry fields are **required** in ``calibration_config.yaml`` under
``pan_tilt_servo`` before running calibration or replay.  Setting
``camera_forward_offset_m`` to ``0.0`` is allowed and makes the correction
reduce to the identity, which is equivalent to the naive/pinhole model.  The
corrected angle is what is stored in ``phi_deg`` in the CSV and fitted to
derive the servo curve.

Must be run on the Raspberry Pi with the UGV rover and Waveshare RGB camera
connected. Intrinsic calibration (``calibrate_waveshare_camera.py``) must have
been completed first.

The ``ugv-follower`` package must be installed before running this script::

    pip install -e ugv-follower/

How to run
----------
::

    ugv-calibrate-pantilt

    # Or via python -m:
    python -m tools.calibration.calibrate_pantilt_servo

    # Offline refit from an existing CSV (no hardware required):
    ugv-calibrate-pantilt --replay calibration/pantilt_servo/<timestamp>.csv
    ugv-calibrate-pantilt --replay calibration/pantilt_servo/<timestamp>.csv --save

Arguments
---------
--replay        Path to an existing raw CSV for offline refit (no hardware).
--save          When used with --replay, write results to sensor_config.yaml.
--camera-device V4L2 device path for the Waveshare RGB camera.
                Overrides calibration_config.yaml pan_tilt_servo.camera_device.
--sensor-config Path to sensor_config.yaml (default: configs/sensor_config.yaml).
--cal-config    Path to calibration_config.yaml (default: configs/calibration_config.yaml).
--noise-floor   Override shared.noise_floor_deg from calibration config.

Hardware setup
--------------
1. Mount the UGV on a flat surface with clear space ahead (≥ 1.5 m).
2. Connect the UGV rover sub-controller to the port listed under ``ugv.port``.
3. Connect the Waveshare RGB camera (USB) — note its device path.
4. Power on the rover.
5. Place a distinct stationary target (e.g. coloured tape on a wall) at the
   expected distance, roughly ahead of the rover.

Calibration procedure
---------------------
1. Run the script.
2. The rover commands the pan-tilt to (0°, tilt_setpoint_deg) and waits.
3. A live MJPEG stream is served at ``http://<pi-ip>:8080/stream``.  Open this
   URL in a browser to see the camera feed with a vertical green guide line at
   column ``cx`` (the principal point from intrinsic calibration).
4. Physically position the rover until the stationary target is bisected by the
   green guide line.
5. ``GET /confirm`` to start the commanded sweep.  The tool will automatically
   sweep pan commands across all configured schedules, pausing at
   each step to collect settled frames.
6. Poll ``GET /status`` until ``state`` is ``COMPLETE`` (or ``FAILED``).
7. ``GET /save`` to write the CSV and update sensor_config.yaml.
8. ``GET /abort`` at any time to cancel without saving.

Endpoints
---------
GET /stream   — MJPEG stream; open in browser to see the live annotated feed.
GET /status   — JSON: {``state``, ``progress_pct``, ``current_step``,
                       ``total_steps``, ``error_message``}.
GET /confirm  — Start the commanded sweep (only valid in WAITING_ZERO).
               Returns {``status``: ``sweeping``} or 409 if in wrong state.
GET /abort    — Cancel any in-progress sweep and shut down.
               Returns {``status``: ``aborted``}.
GET /save     — Write CSV and results to sensor_config.yaml (only valid in
               COMPLETE).  Returns {``saved``, ``csv_path``,
               ``n_good_samples``, ``n_total_samples``, ``hysteresis_mean_deg``}
               or 409.  Idempotent: repeated calls return the cached result.
GET /reset    — Reset state back to WAITING_ZERO (valid from FAILED or
               COMPLETE).  Returns {``status``: ``reset``} or 409 if SWEEPING.
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
from typing import Any

import cv2
import numpy as np
import yaml
from loguru import logger

from ugv_follower.control.ugv_controller import UGVController
from ugv_follower.utils.camera_preflight import ensure_camera_device_available
from ugv_follower.utils.config_utils import get_project_root

_DEFAULT_SENSOR_CONFIG: Path = get_project_root() / "configs" / "sensor_config.yaml"
_DEFAULT_CAL_CONFIG: Path = get_project_root() / "configs" / "calibration_config.yaml"
_CSV_DIR: Path = get_project_root() / "calibration" / "pantilt_servo"

_STREAM_PORT: int = 8080
_JPEG_QUALITY: int = 80

# Pixels from frame edge below which a centroid is flagged as near-edge.
_EDGE_MARGIN_PX: int = 20

# Seconds to wait after commanding pan-tilt to zero before opening the camera.
_SERVO_SETTLE_S: float = 1.5

# Canonical CSV column order.
_CSV_COLUMNS: list[str] = [
    "timestamp_s",
    "commanded_pan_deg",
    "tilt_setpoint_deg",
    "sweep_direction",
    "schedule_index",
    "u_px",
    "fx_px",
    "cx_px",
    "image_angle_deg",
    "phi_deg",
    "settle_time_s",
    "frames_averaged",
    "match_score",
    "quality_flag",
]


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------


class CalibrationState(str, Enum):
    """State machine values for the calibration workflow.

    Using ``str`` as a mixin ensures ``state.value`` serialises to a plain
    string in JSON responses, preserving the existing endpoint contract.
    """

    WAITING_ZERO = "WAITING_ZERO"
    SWEEPING = "SWEEPING"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"
    ABORTED = "ABORTED"


# ---------------------------------------------------------------------------
# Data models
# ---------------------------------------------------------------------------


@dataclass
class SweepStatus:
    """Snapshot of sweep progress.

    Treated as an immutable value: ``CalibrationStateContainer`` replaces the
    whole instance atomically rather than mutating individual fields.
    """

    state: CalibrationState = CalibrationState.WAITING_ZERO
    progress_pct: float = 0.0
    current_step: int = 0
    total_steps: int = 0
    error_message: str | None = None


@dataclass(frozen=True)
class PanTiltCalConfig:
    """Validated, derived configuration for one calibration run.

    Constructed by ``_load_config()`` which reads sensor_config.yaml,
    calibration_config.yaml, and the parsed CLI arguments.
    """

    cx: float
    cy: float
    fx: float
    cam_width: int
    cam_height: int
    ugv_port: str
    ugv_baud: int
    chassis_main: int
    chassis_module: int
    track_width: float
    pan_command_schedules_deg: tuple[tuple[float, ...], ...]
    settle_time_s: float
    frames_to_average: int
    tilt_setpoint_deg: float
    template_half_width_px: int
    noise_floor_deg: float
    sign_override: float | None
    camera_device: str
    sensor_config_path: Path
    camera_forward_offset_m: float
    calibration_target_distance_m: float
    precondition_cycles: int
    precondition_settle_time_s: float  # dwell time used only during warmup passes
    tracking_hysteresis_enter_deg: float
    tracking_hysteresis_exit_deg: float


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
        """Replace the stored annotated frame. Called from the frame thread."""
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
        """Atomically transition WAITING_ZERO → SWEEPING. Returns True on success."""
        with self._lock:
            if self._status.state != CalibrationState.WAITING_ZERO:
                return False
            self._status = SweepStatus(state=CalibrationState.SWEEPING)
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
        """Atomically reset to WAITING_ZERO, clearing previous results.

        Returns True on success, or False if the current state is SWEEPING
        (a reset while sweeping is not permitted).
        """
        with self._lock:
            if self._status.state == CalibrationState.SWEEPING:
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


def centroid_to_angle(u_px: float, cx: float, fx: float) -> float:
    """Convert a measured horizontal centroid to the inferred pan angle (degrees).

    Uses the pinhole camera model inverse::

        phi = -degrees(arctan2(u_px - cx, fx))

    Sign convention (default, sign_override=null):

    - Camera panned right (+cmd) → target shifts left in image (u < cx) →
      ``phi > 0``
    - Camera panned left  (-cmd) → target shifts right in image (u > cx) →
      ``phi < 0``

    The result of this function is multiplied by ``sign_mult`` in the sweep
    thread, where ``sign_mult = sign_override`` if provided, else 1.0.

    Parameters
    ----------
    u_px : float
        Horizontal centroid of the target in pixels.
    cx : float
        Principal point x from intrinsic calibration (pixels).
    fx : float
        Focal length x from intrinsic calibration (pixels).

    Returns
    -------
    float
        Signed pan angle in degrees.
    """
    return -math.degrees(math.atan2(u_px - cx, fx))


def correct_for_forward_offset(phi_cam_deg: float, d_m: float, D_m: float) -> float:
    """Apply forward camera displacement correction to naive camera-frame pan angle.

    Accounts for the camera lens being displaced forward from the pan-tilt rotation
    centre by distance ``d_m``.  Without correction, a forward-displaced camera
    makes the target appear to subtend a slightly larger angle than the true servo
    angle, introducing a systematic slope error in the fitted curve.

    Derivation (2-D horizontal plane):
    Camera at ``(d·sin θ, d·cos θ)``, target at ``(0, D)``::

        phi_corrected = phi_cam − arcsin( d/D · sin(phi_cam) )

    When ``d=0`` the formula reduces to the identity (naive model).

    Parameters
    ----------
    phi_cam_deg : float
        Naive camera-frame pan angle in degrees, as returned by
        ``centroid_to_angle``.
    d_m : float
        Forward offset of the camera lens from the pan-tilt rotation centre
        (metres).  Must be ``>= 0`` and ``< D_m``.
    D_m : float
        Distance from the pan-tilt rotation centre to the calibration target
        (metres).  Must be ``> 0``.

    Returns
    -------
    float
        Offset-corrected pan angle in degrees.

    Raises
    ------
    ValueError
        If the arcsin domain constraint ``|d/D · sin(phi_cam)| < 1`` is
        violated (only possible when ``d >= D``, which ``_validate_geometry``
        prevents during config loading).
    """
    phi_rad = math.radians(phi_cam_deg)
    ratio = (d_m / D_m) * math.sin(phi_rad)
    if abs(ratio) >= 1.0:
        raise ValueError(
            f"Trig domain violation in correct_for_forward_offset: "
            f"d/D*sin(phi_cam) = {ratio:.4f} must be in (-1, 1). "
            f"d={d_m}, D={D_m}, phi_cam={phi_cam_deg:.2f}°"
        )
    return phi_cam_deg - math.degrees(math.asin(ratio))


def quality_flag(
    match_score: float,
    u_px: float,
    cam_width: int,
    edge_margin_px: int = _EDGE_MARGIN_PX,
    min_score: float = 0.5,
) -> int:
    """Compute an integer quality flag for a single centroid measurement.

    Parameters
    ----------
    match_score : float
        ``cv2.matchTemplate`` (TM_CCOEFF_NORMED) score, 0–1.
    u_px : float
        Measured horizontal centroid in pixels.
    cam_width : int
        Frame width in pixels.
    edge_margin_px : int
        Pixels from frame edge below which near_edge=True.
    min_score : float
        Score threshold below which low_match=True.

    Returns
    -------
    int
        0=good, 1=low_match, 2=near_edge, 3=both.
    """
    low_match = match_score < min_score
    near_edge = u_px < edge_margin_px or u_px > (cam_width - edge_margin_px)
    if low_match and near_edge:
        return 3
    if near_edge:
        return 2
    if low_match:
        return 1
    return 0


def estimate_dead_band(
    cmds: list[float],
    phis: list[float],
    noise_floor_deg: float,
) -> tuple[float | None, float | None]:
    """Estimate positive and negative dead-band command boundaries.

    Scans the command-angle pairs to find the smallest positive command
    magnitude where |phi| > noise_floor_deg, and the smallest negative command
    magnitude (closest to zero) where the same condition holds.

    Parameters
    ----------
    cmds : list[float]
        Commanded pan values (degrees), any order.
    phis : list[float]
        Corresponding measured angles (degrees).
    noise_floor_deg : float
        Threshold below which motion is considered noise.

    Returns
    -------
    (dead_band_pos_deg, dead_band_neg_deg)
        Each is the boundary command value, or None if no motion is detected on
        that side.
    """
    pos_candidates = sorted(
        [(c, p) for c, p in zip(cmds, phis) if c > 0],
        key=lambda x: x[0],  # ascending: smallest positive first
    )
    neg_candidates = sorted(
        [(c, p) for c, p in zip(cmds, phis) if c < 0],
        key=lambda x: -x[0],  # ascending absolute value: closest to zero first
    )

    dead_band_pos: float | None = None
    for c, p in pos_candidates:
        if abs(p) > noise_floor_deg:
            dead_band_pos = c
            break

    dead_band_neg: float | None = None
    for c, p in neg_candidates:
        if abs(p) > noise_floor_deg:
            dead_band_neg = c
            break

    return dead_band_pos, dead_band_neg


def fit_linear(
    cmds: list[float],
    phis: list[float],
) -> dict[str, float]:
    """Fit phi ≈ slope * cmd + intercept using ``np.polyfit``.

    Parameters
    ----------
    cmds : list[float]
        Commanded values.
    phis : list[float]
        Measured angles (degrees).

    Returns
    -------
    dict
        ``{slope, intercept, mae_deg, max_abs_error_deg}``.
    """
    if len(cmds) < 2:
        logger.warning("fit_linear: fewer than 2 samples — returning zero fit.")
        return {
            "slope": 0.0,
            "intercept": 0.0,
            "mae_deg": 0.0,
            "max_abs_error_deg": 0.0,
        }
    cmds_arr = np.array(cmds, dtype=float)
    phis_arr = np.array(phis, dtype=float)
    slope, intercept = np.polyfit(cmds_arr, phis_arr, 1)
    predicted = slope * cmds_arr + intercept
    residuals = np.abs(phis_arr - predicted)
    return {
        "slope": float(slope),
        "intercept": float(intercept),
        "mae_deg": float(np.mean(residuals)),
        "max_abs_error_deg": float(np.max(residuals)),
    }


def fit_piecewise_linear(
    cmds: list[float],
    phis: list[float],
) -> dict[str, Any]:
    """Build a sorted (cmd, phi) interpolation table and compute leave-one-out metrics.

    The piecewise-linear model is simply the sorted sample pairs.  Fit metrics
    are computed via leave-one-out: for each sample i, ``np.interp`` is evaluated
    at ``cmds[i]`` using all other samples, and the residual against ``phis[i]``
    is recorded.

    Parameters
    ----------
    cmds : list[float]
        Commanded values (any order).
    phis : list[float]
        Measured angles (degrees).

    Returns
    -------
    dict
        ``{command_samples, angle_samples, mae_deg, max_abs_error_deg}``.
        If fewer than 3 samples are provided, metrics are set to 0.0 with a
        warning.
    """
    if len(cmds) < 2:
        logger.warning("fit_piecewise_linear: fewer than 2 samples.")
        pairs = sorted(zip(cmds, phis), key=lambda x: x[0])
        return {
            "command_samples": [p[0] for p in pairs],
            "angle_samples": [p[1] for p in pairs],
            "mae_deg": 0.0,
            "max_abs_error_deg": 0.0,
        }

    pairs = sorted(zip(cmds, phis), key=lambda x: x[0])
    sorted_cmds = [p[0] for p in pairs]
    sorted_phis = [p[1] for p in pairs]

    if len(cmds) < 3:
        logger.warning(
            f"fit_piecewise_linear: only {len(cmds)} samples — "
            "leave-one-out metrics set to 0.0."
        )
        return {
            "command_samples": sorted_cmds,
            "angle_samples": sorted_phis,
            "mae_deg": 0.0,
            "max_abs_error_deg": 0.0,
        }

    cmds_arr = np.array(sorted_cmds, dtype=float)
    phis_arr = np.array(sorted_phis, dtype=float)

    residuals: list[float] = []
    for i in range(len(sorted_cmds)):
        mask = np.ones(len(sorted_cmds), dtype=bool)
        mask[i] = False
        pred = float(np.interp(cmds_arr[i], cmds_arr[mask], phis_arr[mask]))
        residuals.append(abs(float(phis_arr[i]) - pred))

    return {
        "command_samples": sorted_cmds,
        "angle_samples": sorted_phis,
        "mae_deg": float(np.mean(residuals)),
        "max_abs_error_deg": float(np.max(residuals)),
    }


def compute_hysteresis(
    fwd_cmds: list[float],
    fwd_phis: list[float],
    rev_cmds: list[float],
    rev_phis: list[float],
) -> float:
    """Compute mean absolute hysteresis across matched command values.

    For each command value present in both the forward and reverse sweeps,
    computes |phi_fwd − phi_rev| and returns the mean.

    Parameters
    ----------
    fwd_cmds : list[float]
        Forward sweep commanded values.
    fwd_phis : list[float]
        Forward sweep measured angles.
    rev_cmds : list[float]
        Reverse sweep commanded values.
    rev_phis : list[float]
        Reverse sweep measured angles.

    Returns
    -------
    float
        Mean absolute hysteresis in degrees (0.0 if no matching commands).
    """
    rev_map = {c: p for c, p in zip(rev_cmds, rev_phis)}
    diffs = [
        abs(p_fwd - rev_map[c]) for c, p_fwd in zip(fwd_cmds, fwd_phis) if c in rev_map
    ]
    return float(np.mean(diffs)) if diffs else 0.0


def analyse_sweep(
    rows: list[dict[str, Any]],
    noise_floor_deg: float,
    camera_forward_offset_m: float,
    calibration_target_distance_m: float,
) -> dict[str, Any]:
    """Compute all model outputs from raw sweep rows.

    Filters to ``quality_flag == 0``, splits into forward / reverse sweeps,
    runs all fit helpers, and assembles the complete ``pan_tilt_servo`` dict
    ready to be merged into sensor_config.yaml.

    This is the single function called by both the live ``/save`` handler and
    ``--replay`` mode, so all offline analysis exactly mirrors the live path.
    ``phi_deg`` values in ``rows`` are assumed to be already offset-corrected
    (as written by ``_run_sweep``).

    Parameters
    ----------
    rows : list[dict]
        Raw sweep rows as produced by the sweep thread (or loaded via
        ``_load_csv``).
    noise_floor_deg : float
        Passed through to ``estimate_dead_band``.
    camera_forward_offset_m : float
        Forward offset of camera lens from rotation centre (metres).
        Recorded as metadata in the output dict.
    calibration_target_distance_m : float
        Distance from rotation centre to calibration target (metres).
        Recorded as metadata in the output dict.

    Returns
    -------
    dict
        Complete ``pan_tilt_servo`` section (without ``calibrated_at`` and
        ``raw_csv`` — the caller fills those in).
    """
    good = [r for r in rows if int(r["quality_flag"]) == 0]
    n_good = len(good)
    n_total = len(rows)

    if n_good == 0:
        logger.error("No good-quality samples (quality_flag==0) found in sweep data.")
        return {
            "n_good_samples": 0,
            "n_total_samples": n_total,
            "error": "no_good_samples",
        }

    fwd_good = [r for r in good if r["sweep_direction"] == "forward"]
    rev_good = [r for r in good if r["sweep_direction"] == "reverse"]

    all_cmds = [float(r["commanded_pan_deg"]) for r in good]
    all_phis = [float(r["phi_deg"]) for r in good]
    fwd_cmds = [float(r["commanded_pan_deg"]) for r in fwd_good]
    fwd_phis = [float(r["phi_deg"]) for r in fwd_good]
    rev_cmds = [float(r["commanded_pan_deg"]) for r in rev_good]
    rev_phis = [float(r["phi_deg"]) for r in rev_good]

    dead_band_pos, dead_band_neg = estimate_dead_band(
        all_cmds, all_phis, noise_floor_deg
    )
    linear = fit_linear(all_cmds, all_phis)
    pw_fwd = fit_piecewise_linear(fwd_cmds, fwd_phis)
    pw_rev = fit_piecewise_linear(rev_cmds, rev_phis)

    # For the combined piecewise fit, deduplicate command values by averaging
    # the measured angles across forward and reverse sweeps. This yields a
    # single-valued cmd→phi mapping suitable for interpolation.
    cmd_to_phis: dict[float, list[float]] = {}
    for cmd, phi in zip(all_cmds, all_phis):
        cmd_to_phis.setdefault(cmd, []).append(phi)

    combined_cmds = sorted(cmd_to_phis)
    combined_phis = [float(np.median(cmd_to_phis[cmd])) for cmd in combined_cmds]
    pw_combined = fit_piecewise_linear(combined_cmds, combined_phis)
    hysteresis = compute_hysteresis(fwd_cmds, fwd_phis, rev_cmds, rev_phis)

    phi_min = float(min(all_phis))
    phi_max = float(max(all_phis))
    sweep_min = float(min(all_cmds))
    sweep_max = float(max(all_cmds))

    tilt_sp = float(good[0]["tilt_setpoint_deg"]) if good else 0.0

    return {
        "calibration_method": "servo_curve_sweep",
        "angle_measurement_model": "offset_corrected_forward_displacement",
        "camera_forward_offset_m": camera_forward_offset_m,
        "calibration_target_distance_m": calibration_target_distance_m,
        "tilt_setpoint_deg": tilt_sp,
        "sweep_min_deg": sweep_min,
        "sweep_max_deg": sweep_max,
        "dead_band_pos_deg": round(dead_band_pos, 4)
        if dead_band_pos is not None
        else None,
        "dead_band_neg_deg": round(dead_band_neg, 4)
        if dead_band_neg is not None
        else None,
        "cmd_min": sweep_min,
        "cmd_max": sweep_max,
        "phi_min_deg": round(phi_min, 4),
        "phi_max_deg": round(phi_max, 4),
        "hysteresis_mean_deg": round(hysteresis, 4),
        "linear_fit": {
            "slope": round(linear["slope"], 6),
            "intercept": round(linear["intercept"], 6),
            "mae_deg": round(linear["mae_deg"], 4),
            "max_abs_error_deg": round(linear["max_abs_error_deg"], 4),
        },
        "piecewise_linear": {
            "forward": {
                "command_samples": pw_fwd["command_samples"],
                "angle_samples": [round(v, 4) for v in pw_fwd["angle_samples"]],
            },
            "reverse": {
                "command_samples": pw_rev["command_samples"],
                "angle_samples": [round(v, 4) for v in pw_rev["angle_samples"]],
            },
            "combined": {
                "command_samples": pw_combined["command_samples"],
                "angle_samples": [round(v, 4) for v in pw_combined["angle_samples"]],
            },
            "mae_deg": round(pw_combined["mae_deg"], 4),
            "max_abs_error_deg": round(pw_combined["max_abs_error_deg"], 4),
        },
        "n_good_samples": n_good,
        "n_total_samples": n_total,
    }


# ---------------------------------------------------------------------------
# Config parsing
# ---------------------------------------------------------------------------


def _validate_schedules(schedules: list[list[float]]) -> None:
    """Raise ValueError if any schedule is empty or not strictly monotonic ascending.

    Parameters
    ----------
    schedules : list[list[float]]
        The parsed pan command schedules to validate.

    Raises
    ------
    ValueError
        If schedules is empty, any schedule has fewer than 2 values, or any
        schedule is not strictly monotonic ascending.
    """
    if not schedules:
        raise ValueError(
            "pan_tilt_servo.pan_command_schedules_deg must contain at least one schedule."
        )
    for i, sched in enumerate(schedules):
        if len(sched) < 2:
            raise ValueError(
                f"Schedule {i} must have at least 2 values, got {len(sched)}."
            )
        for j in range(len(sched) - 1):
            if sched[j] >= sched[j + 1]:
                raise ValueError(
                    f"Schedule {i} is not strictly monotonic ascending at index {j}: "
                    f"{sched[j]} >= {sched[j + 1]}."
                )


def _validate_geometry(
    camera_forward_offset_m: float,
    calibration_target_distance_m: float,
) -> None:
    """Validate pan-tilt geometry parameters required for offset-corrected calibration.

    Raises
    ------
    ValueError
        If any constraint is violated.
    """
    if calibration_target_distance_m <= 0:
        raise ValueError(
            f"calibration_target_distance_m must be positive, "
            f"got {calibration_target_distance_m}"
        )
    if camera_forward_offset_m < 0:
        raise ValueError(
            f"camera_forward_offset_m must be non-negative, "
            f"got {camera_forward_offset_m}"
        )
    if camera_forward_offset_m >= calibration_target_distance_m:
        raise ValueError(
            f"camera_forward_offset_m ({camera_forward_offset_m}) must be less than "
            f"calibration_target_distance_m ({calibration_target_distance_m})"
        )


def _extract_intrinsics(cfg: dict[str, Any]) -> tuple[float, float, float]:
    """Return (cx, cy, fx) from ``waveshare_rgb.camera_matrix``.

    Raises
    ------
    ValueError
        If ``camera_matrix`` is null (intrinsic calibration not yet run).
    """
    matrix = cfg.get("waveshare_rgb", {}).get("camera_matrix")
    if matrix is None:
        raise ValueError(
            "waveshare_rgb.camera_matrix is null in sensor_config.yaml. "
            "Run calibrate_waveshare_camera.py first."
        )
    cx = float(matrix[0][2])
    cy = float(matrix[1][2])
    fx = float(matrix[0][0])
    return cx, cy, fx


def _load_config(
    sensor_cfg: dict[str, Any],
    cal_cfg: dict[str, Any],
    args: argparse.Namespace,
) -> PanTiltCalConfig:
    """Parse and validate both config dicts + CLI args into a ``PanTiltCalConfig``.

    Raises
    ------
    ValueError
        If required fields are missing, null, or out of range.
    KeyError
        If required top-level sections are absent from the sensor config.
    """
    cx, cy, fx = _extract_intrinsics(sensor_cfg)

    res = sensor_cfg.get("waveshare_rgb", {}).get("resolution", [1280, 720])
    cam_width, cam_height = int(res[0]), int(res[1])

    ugv_cfg: dict[str, Any] = sensor_cfg["ugv"]
    pt_cfg: dict[str, Any] = cal_cfg.get("pan_tilt_servo", {})
    shared_cfg: dict[str, Any] = cal_cfg.get("shared", {})

    settle_time = float(
        shared_cfg.get("settle_time_s", pt_cfg.get("settle_time_s", 1.5))
    )
    frames_to_avg = int(pt_cfg.get("frames_to_average", 10))
    tilt_sp = float(pt_cfg.get("tilt_setpoint_deg", 0.0))
    tmpl_half_w = int(
        shared_cfg.get(
            "template_half_width_px", pt_cfg.get("template_half_width_px", 60)
        )
    )
    noise_floor = float(
        shared_cfg.get("noise_floor_deg", pt_cfg.get("noise_floor_deg", 0.5))
    )

    precondition_cycles_raw = int(pt_cfg.get("precondition_cycles", 0))
    if precondition_cycles_raw < 0:
        raise ValueError("pan_tilt_servo.precondition_cycles must be >= 0.")

    # Falls back to settle_time so existing configs behave identically.
    precondition_settle_time = float(
        pt_cfg.get("precondition_settle_time_s", settle_time)
    )
    if precondition_settle_time < 0:
        raise ValueError(
            "pan_tilt_servo.precondition_settle_time_s must be non-negative."
        )

    tracking_hysteresis_enter = float(
        pt_cfg.get("tracking_hysteresis_enter_deg", 1.5)
    )
    tracking_hysteresis_exit = float(pt_cfg.get("tracking_hysteresis_exit_deg", 3.0))
    if tracking_hysteresis_enter < 0:
        raise ValueError(
            "pan_tilt_servo.tracking_hysteresis_enter_deg must be >= 0."
        )
    if tracking_hysteresis_exit < tracking_hysteresis_enter:
        raise ValueError(
            "pan_tilt_servo.tracking_hysteresis_exit_deg must be >= "
            "pan_tilt_servo.tracking_hysteresis_enter_deg."
        )

    sign_override_raw = pt_cfg.get("sign_override")
    sign_override: float | None = (
        None if sign_override_raw is None else float(sign_override_raw)
    )

    # CLI --camera-device overrides config; config overrides default.
    camera_device = args.camera_device or str(
        shared_cfg.get("camera_device", pt_cfg.get("camera_device", "/dev/video0"))
    )

    # CLI --noise-floor overrides config.
    if args.noise_floor is not None:
        noise_floor = float(args.noise_floor)

    # Required geometry fields for offset-corrected angle estimation.
    _fwd_raw = pt_cfg.get("camera_forward_offset_m")
    if _fwd_raw is None:
        raise ValueError(
            "pan_tilt_servo.camera_forward_offset_m is missing from calibration_config.yaml. "
            "Add it before running calibration."
        )
    camera_fwd_offset = float(_fwd_raw)

    _dist_raw = pt_cfg.get("calibration_target_distance_m")
    if _dist_raw is None:
        raise ValueError(
            "pan_tilt_servo.calibration_target_distance_m is missing from calibration_config.yaml. "
            "Add it before running calibration."
        )
    cal_target_dist = float(_dist_raw)

    _validate_geometry(camera_fwd_offset, cal_target_dist)

    sensor_config_path: Path = args.sensor_config.resolve()

    schedules_raw = pt_cfg.get("pan_command_schedules_deg")
    if not schedules_raw:
        raise ValueError(
            "pan_tilt_servo.pan_command_schedules_deg is missing or empty "
            "in calibration_config.yaml."
        )
    schedules: list[list[float]] = [[float(v) for v in s] for s in schedules_raw]
    _validate_schedules(schedules)
    pan_command_schedules = tuple(tuple(s) for s in schedules)

    return PanTiltCalConfig(
        cx=cx,
        cy=cy,
        fx=fx,
        cam_width=cam_width,
        cam_height=cam_height,
        ugv_port=str(ugv_cfg["port"]),
        ugv_baud=int(ugv_cfg["baud_rate"]),
        chassis_main=int(ugv_cfg["chassis_main"]),
        chassis_module=int(ugv_cfg["chassis_module"]),
        track_width=float(ugv_cfg["track_width"]),
        pan_command_schedules_deg=pan_command_schedules,
        settle_time_s=settle_time,
        frames_to_average=frames_to_avg,
        tilt_setpoint_deg=tilt_sp,
        template_half_width_px=tmpl_half_w,
        noise_floor_deg=noise_floor,
        sign_override=sign_override,
        camera_device=camera_device,
        sensor_config_path=sensor_config_path,
        camera_forward_offset_m=camera_fwd_offset,
        calibration_target_distance_m=cal_target_dist,
        precondition_cycles=precondition_cycles_raw,
        precondition_settle_time_s=precondition_settle_time,
        tracking_hysteresis_enter_deg=tracking_hysteresis_enter,
        tracking_hysteresis_exit_deg=tracking_hysteresis_exit,
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
    cap    : cv2.VideoCapture  Opened camera.
    cx     : float             Principal point x (pixels).
    cy     : float             Principal point y (pixels).
    half_w : int               Half-width of the square template (pixels).

    Returns
    -------
    cv2.typing.MatLike
        The cropped template patch.

    Raises
    ------
    RuntimeError
        If the camera returns no frame.
    """
    ok, frame = cap.read()
    if not ok or frame is None:
        raise RuntimeError("Failed to capture frame for template extraction.")
    cx_i = int(round(cx))
    cy_i = int(round(cy))
    y1 = max(0, cy_i - half_w)
    y2 = min(frame.shape[0], cy_i + half_w)
    x1 = max(0, cx_i - half_w)
    x2 = min(frame.shape[1], cx_i + half_w)
    return frame[y1:y2, x1:x2].copy()


def _match_template(
    frame: cv2.typing.MatLike,
    template: cv2.typing.MatLike,
) -> tuple[float, float]:
    """Run TM_CCOEFF_NORMED template matching and return centroid and score.

    Parameters
    ----------
    frame    : MatLike  Full camera frame.
    template : MatLike  Template patch (as captured by ``_capture_template``).

    Returns
    -------
    (centroid_u_px, match_score)
        ``centroid_u_px`` is the horizontal centre of the best match location.
        ``match_score`` is the normalised correlation coefficient (0–1).
    """
    result = cv2.matchTemplate(frame, template, cv2.TM_CCOEFF_NORMED)
    _, max_val, _, max_loc = cv2.minMaxLoc(result)
    centroid_u = max_loc[0] + template.shape[1] / 2.0
    return centroid_u, float(max_val)


# ---------------------------------------------------------------------------
# CSV I/O
# ---------------------------------------------------------------------------


def _make_csv_path() -> Path:
    """Generate a timestamped CSV path under ``_CSV_DIR``.

    Example: ``calibration/pantilt_servo/20260331T123456.csv``
    """
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


_CSV_INT_COLS: frozenset[str] = frozenset(
    {"frames_averaged", "quality_flag", "schedule_index"}
)
_CSV_STR_COLS: frozenset[str] = frozenset({"sweep_direction"})


def _load_csv(path: Path) -> list[dict[str, Any]]:
    """Load a CSV written by ``_write_csv`` and return rows as typed dicts.

    Numeric fields are cast to ``float``; integer columns to ``int``;
    string columns remain ``str``.
    """
    rows: list[dict[str, Any]] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        has_schedule_index = (
            reader.fieldnames is not None and "schedule_index" in reader.fieldnames
        )
        for row in reader:
            typed: dict[str, Any] = {}
            for k, v in row.items():
                if k in _CSV_INT_COLS:
                    typed[k] = int(v)
                elif k in _CSV_STR_COLS:
                    typed[k] = v
                else:
                    typed[k] = float(v)
            if not has_schedule_index:
                typed["schedule_index"] = 0
            rows.append(typed)
    return rows


# ---------------------------------------------------------------------------
# Sweep thread
# ---------------------------------------------------------------------------


def _run_precondition_cycles(
    full_steps: list[tuple[float, str]],
    config: PanTiltCalConfig,
    ugv: UGVController,
    cancel_event: threading.Event,
) -> bool:
    """Run warmup cycles to condition the servo before the measurement sweep.

    Commands the pan-tilt through ``config.precondition_cycles`` full
    forward+reverse passes without recording any data.  Raises on hardware
    failure so the caller's outer exception handler can transition to FAILED.

    Parameters
    ----------
    full_steps : list[tuple[float, str]]
        Ordered list of ``(cmd_deg, direction)`` pairs covering the full sweep
        range in both directions.
    config : PanTiltCalConfig
        Calibration configuration (used for tilt setpoint and
        ``precondition_settle_time_s`` dwell time).
    ugv : UGVController
        Connected UGV hardware interface.
    cancel_event : threading.Event
        Set by the ``/abort`` handler to request early termination.

    Returns
    -------
    bool
        ``True`` if all cycles completed normally; ``False`` if cancelled.
    """
    logger.info(
        "Preconditioning: %d warmup cycle(s) over %d steps.",
        config.precondition_cycles,
        len(full_steps),
    )
    for cycle in range(config.precondition_cycles):
        logger.info(
            "Warmup cycle %d/%d — start.",
            cycle + 1,
            config.precondition_cycles,
        )
        for cmd_deg, _direction in full_steps:
            if cancel_event.is_set():
                logger.info("Sweep cancelled during preconditioning.")
                return False
            ugv.set_pan_tilt(cmd_deg, config.tilt_setpoint_deg)
            time.sleep(config.precondition_settle_time_s)
            if cancel_event.is_set():
                logger.info("Sweep cancelled during preconditioning settle.")
                return False
        logger.info(
            "Warmup cycle %d/%d — complete.",
            cycle + 1,
            config.precondition_cycles,
        )
    logger.info("Preconditioning complete. Starting measurement sweep.")
    return True


def _capture_frames(
    cap: cv2.VideoCapture,
    template: cv2.typing.MatLike,
    config: PanTiltCalConfig,
    cap_lock: threading.Lock,
) -> tuple[list[float], list[float]]:
    """Capture ``frames_to_average`` frames under the camera lock and return measurements.

    Acquires ``cap_lock`` exclusively (blocking the frame-display thread) for
    the duration of frame collection.  Failed reads are skipped with a warning
    rather than raising, so callers should check whether the returned lists are
    non-empty.

    Parameters
    ----------
    cap : cv2.VideoCapture
        Open camera capture device.
    template : cv2.typing.MatLike
        Template image used by ``_match_template``.
    config : PanTiltCalConfig
        Provides ``frames_to_average``.
    cap_lock : threading.Lock
        Shared lock that serialises camera access between this thread and the
        frame-display thread.

    Returns
    -------
    u_list : list[float]
        Horizontal centroid pixel coordinates, one per successfully read frame.
    score_list : list[float]
        Template-match scores corresponding to each entry in ``u_list``.
    """
    u_list: list[float] = []
    score_list: list[float] = []
    with cap_lock:
        for _ in range(config.frames_to_average):
            ok, frame = cap.read()
            if not ok or frame is None:
                logger.warning("cap.read() failed during sweep — skipping frame.")
                continue
            u, score = _match_template(frame, template)
            u_list.append(u)
            score_list.append(score)
    return u_list, score_list


def _build_sweep_row(
    cmd_deg: float,
    direction: str,
    schedule_index: int,
    u_median: float,
    score_med: float,
    qflag: int,
    phi: float,
    image_angle_deg: float,
    config: PanTiltCalConfig,
    t0: float,
    frames_captured: int,
) -> dict[str, Any]:
    """Construct a single sweep measurement row dict.

    All floating-point values are rounded to match the CSV precision used by
    ``_write_csv`` / ``_load_csv``.

    Parameters
    ----------
    cmd_deg : float
        Commanded pan angle in degrees.
    direction : str
        Sweep direction label (``"forward"`` or ``"reverse"``).
    schedule_index : int
        Zero-based index of the schedule this step belongs to.
    u_median : float
        Median horizontal centroid in pixels.
    score_med : float
        Median template-match score.
    qflag : int
        Quality flag from ``quality_flag()``.
    phi : float
        Offset-corrected pan angle in degrees.
    image_angle_deg : float
        Naive image-plane angle in degrees (arctan of pixel offset).
    config : PanTiltCalConfig
        Provides ``fx``, ``cx``, ``tilt_setpoint_deg``, and ``settle_time_s``.
    t0 : float
        ``time.monotonic()`` reference at sweep start.
    frames_captured : int
        Number of frames that were successfully read and averaged.

    Returns
    -------
    dict[str, Any]
        Row dict ready to be appended to the sweep rows list.
    """
    return {
        "timestamp_s": round(time.monotonic() - t0, 4),
        "commanded_pan_deg": round(cmd_deg, 4),
        "tilt_setpoint_deg": round(config.tilt_setpoint_deg, 4),
        "sweep_direction": direction,
        "schedule_index": schedule_index,
        "u_px": round(u_median, 3),
        "fx_px": round(config.fx, 4),
        "cx_px": round(config.cx, 4),
        "image_angle_deg": round(image_angle_deg, 6),
        "phi_deg": round(phi, 6),
        "settle_time_s": round(config.settle_time_s, 4),
        "frames_averaged": frames_captured,
        "match_score": round(score_med, 6),
        "quality_flag": qflag,
    }


def _run_sweep(
    cap: cv2.VideoCapture,
    ugv: UGVController,
    config: PanTiltCalConfig,
    template: cv2.typing.MatLike,
    state: CalibrationStateContainer,
    cap_lock: threading.Lock,
    cancel_event: threading.Event,
    t0: float,
) -> None:
    """Execute the full forward + reverse sweep and transition to COMPLETE or FAILED.

    Intended to run in a daemon thread.  For each commanded step:

    1. Command the UGV pan-tilt.
    2. Sleep ``settle_time_s``.
    3. Collect ``frames_to_average`` frames under ``cap_lock`` (exclusive with
       the frame thread).
    4. Median-aggregate centroids and match scores.
    5. Compute ``quality_flag`` and ``phi_deg``.
    6. Append a row dict.

    On completion: calls ``state.set_sweep_rows`` then transitions to COMPLETE.
    On cancellation: returns without transitioning (the HTTP ``/abort`` handler
    transitions to ABORTED).
    On exception: transitions to FAILED.

    Parameters
    ----------
    t0 : float
        ``time.monotonic()`` reference at sweep start, used for ``timestamp_s``.
    """
    total_steps = sum(len(sched) * 2 for sched in config.pan_command_schedules_deg)
    sign_mult = config.sign_override if config.sign_override is not None else 1.0
    rows: list[dict[str, Any]] = []

    if config.precondition_cycles > 0:
        first_sched = config.pan_command_schedules_deg[0]
        precondition_steps: list[tuple[float, str]] = [
            (c, "forward") for c in first_sched
        ] + [(c, "reverse") for c in reversed(first_sched)]
        if not _run_precondition_cycles(precondition_steps, config, ugv, cancel_event):
            return  # cancelled

    try:
        global_step = 0
        for sched_idx, schedule in enumerate(config.pan_command_schedules_deg):
            forward_steps = [(c, "forward") for c in schedule]
            reverse_steps = [(c, "reverse") for c in reversed(schedule)]
            full_steps = forward_steps + reverse_steps

            logger.info(
                "Starting schedule %d/%d (%d steps × 2 dirs).",
                sched_idx + 1,
                len(config.pan_command_schedules_deg),
                len(schedule),
            )

            for cmd_deg, direction in full_steps:
                if cancel_event.is_set():
                    logger.info("Sweep cancelled.")
                    return

                ugv.set_pan_tilt(cmd_deg, config.tilt_setpoint_deg)
                time.sleep(config.settle_time_s)

                if cancel_event.is_set():
                    logger.info("Sweep cancelled during settle.")
                    return

                u_list, score_list = _capture_frames(cap, template, config, cap_lock)

                if not u_list:
                    err = (
                        f"No frames captured at cmd={cmd_deg}° (schedule {sched_idx})."
                    )
                    logger.error(err)
                    state.transition_to(
                        SweepStatus(state=CalibrationState.FAILED, error_message=err)
                    )
                    return

                u_median = float(np.median(u_list))
                score_med = float(np.median(score_list))
                qflag = quality_flag(score_med, u_median, config.cam_width)

                image_angle_deg = math.degrees(
                    math.atan2(u_median - config.cx, config.fx)
                )
                phi_cam = centroid_to_angle(u_median, config.cx, config.fx) * sign_mult
                phi = correct_for_forward_offset(
                    phi_cam,
                    config.camera_forward_offset_m,
                    config.calibration_target_distance_m,
                )

                rows.append(
                    _build_sweep_row(
                        cmd_deg=cmd_deg,
                        direction=direction,
                        schedule_index=sched_idx,
                        u_median=u_median,
                        score_med=score_med,
                        qflag=qflag,
                        phi=phi,
                        image_angle_deg=image_angle_deg,
                        config=config,
                        t0=t0,
                        frames_captured=len(u_list),
                    )
                )

                global_step += 1
                state.update_sweep_progress(global_step, total_steps)
                logger.info(
                    f"Step {global_step}/{total_steps}: "
                    f"sched={sched_idx}  cmd={cmd_deg:+.1f}° ({direction:7s})  "
                    f"u={u_median:.1f}px  phi={phi:+.2f}°  score={score_med:.3f}  flag={qflag}"
                )

        state.set_sweep_rows(rows)
        state.transition_to(
            SweepStatus(
                state=CalibrationState.COMPLETE,
                progress_pct=100.0,
                current_step=total_steps,
                total_steps=total_steps,
            )
        )
        logger.info(f"Sweep complete — {len(rows)} samples collected.")

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
    pan_tilt_servo_dict: dict[str, Any],
) -> None:
    """Merge ``pan_tilt_servo`` section into sensor_config.yaml atomically.

    Reads the existing file, sets (or replaces) the top-level ``pan_tilt_servo``
    key, and re-serialises via ``tempfile`` + ``os.replace``.  All other
    top-level keys are preserved.

    Parameters
    ----------
    sensor_config_path : Path
        Path to ``sensor_config.yaml``.
    pan_tilt_servo_dict : dict
        Calibration results to write under the ``pan_tilt_servo`` key.
    """
    with sensor_config_path.open() as f:
        config: dict[str, Any] = yaml.safe_load(f) or {}

    config["pan_tilt_servo"] = pan_tilt_servo_dict

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
        tmp_path = None  # os.replace succeeded; no cleanup needed
    finally:
        if tmp_path is not None:
            tmp_path.unlink(missing_ok=True)

    logger.info(f"Results written to {sensor_config_path}")


# ---------------------------------------------------------------------------
# Calibration orchestrator
# ---------------------------------------------------------------------------


class CalibrationOrchestrator:
    """Owns calibration logic: starting sweeps and persisting results.

    Decoupled from HTTP transport — has no knowledge of request/response
    handling.

    Parameters
    ----------
    config   : PanTiltCalConfig
    ugv      : UGVController
    cap      : cv2.VideoCapture
    state    : CalibrationStateContainer
    cap_lock : threading.Lock
        Shared lock that serialises camera access between this orchestrator's
        sweep thread and the frame display thread.
    """

    def __init__(
        self,
        config: PanTiltCalConfig,
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
        self._csv_path: Path | None = None
        self._saved_result: dict[str, Any] | None = None

    def confirm_zero(self) -> None:
        """Transition WAITING_ZERO → SWEEPING, capture template, spawn sweep thread.

        Raises
        ------
        RuntimeError
            If not currently in WAITING_ZERO state.
        """
        if not self._state.try_start_sweep():
            raise RuntimeError("not in WAITING_ZERO state")

        logger.info("Capturing template at current pan-tilt zero position...")
        with self._cap_lock:
            template = _capture_template(
                self._cap,
                self._config.cx,
                self._config.cy,
                self._config.template_half_width_px,
            )

        self._cancel_event.clear()
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
                t0,
            ),
            daemon=True,
        ).start()
        logger.info(f"Sweep thread started. CSV target: {self._csv_path}")

    def cancel_sweep(self) -> None:
        """Signal the running sweep thread (if any) to stop early."""
        self._cancel_event.set()

    def save_results(self) -> tuple[Path, dict[str, Any]]:
        """Write CSV and sensor_config.yaml from the completed sweep.

        Idempotent: if results have already been saved, returns the cached
        (csv_path, model) without re-writing.

        Returns
        -------
        (csv_path, model)
            ``csv_path``: Path where the CSV was written.
            ``model``: The ``pan_tilt_servo`` dict written to sensor_config.yaml.

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

        model = analyse_sweep(
            rows,
            self._config.noise_floor_deg,
            self._config.camera_forward_offset_m,
            self._config.calibration_target_distance_m,
        )
        model["tracking_hysteresis_enter_deg"] = (
            self._config.tracking_hysteresis_enter_deg
        )
        model["tracking_hysteresis_exit_deg"] = self._config.tracking_hysteresis_exit_deg
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
        def log_message(
            self, format: str, *args: object
        ) -> None:  # silence access log  # noqa: A002
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
                pass  # client disconnected

        def _handle_confirm(self) -> None:
            try:
                orchestrator.confirm_zero()
            except RuntimeError:
                self._serve_json({"error": "not in WAITING_ZERO state"}, 409)
                return
            self._serve_json({"status": "sweeping"})

        def _handle_abort(self) -> None:
            orchestrator.cancel_sweep()
            state.transition_to(SweepStatus(state=CalibrationState.ABORTED))
            srv = server_ref[0]
            if srv is not None:
                # server.shutdown() blocks; run in a daemon thread so this
                # handler can return its response first.
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
                    "n_good_samples": model.get("n_good_samples"),
                    "n_total_samples": model.get("n_total_samples"),
                    "hysteresis_mean_deg": model.get("hysteresis_mean_deg"),
                }
            )

        def _handle_reset(self) -> None:
            if not state.try_reset():
                self._serve_json({"error": "cannot reset while SWEEPING"}, 409)
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
    """Return the overlay text, BGR colour, and font scale for the given calibration state.

    Returns ``None`` for states that require no status overlay (e.g. IDLE or
    any unrecognised state value).

    Parameters
    ----------
    current_state : str
        The ``CalibrationState.value`` string for the current state.
    status : dict[str, Any]
        Full status dict from ``CalibrationStateContainer.get_status()``;
        used to extract progress fields for the SWEEPING state.

    Returns
    -------
    tuple[str, tuple[int, int, int], float] or None
        ``(text, bgr_colour, font_scale)`` if an overlay should be drawn,
        ``None`` otherwise.
    """
    if current_state == CalibrationState.WAITING_ZERO.value:
        return "Align target to green line, then GET /confirm", (0, 255, 0), 0.8
    if current_state == CalibrationState.SWEEPING.value:
        step = status["current_step"]
        total = status["total_steps"]
        pct = status["progress_pct"]
        return f"Sweeping: {step}/{total}  ({pct:.0f}%)", (0, 200, 255), 0.8
    if current_state == CalibrationState.COMPLETE.value:
        return "COMPLETE — GET /save to persist results", (0, 255, 128), 0.8
    if current_state == CalibrationState.FAILED.value:
        err = status.get("error_message") or "FAILED"
        return f"FAILED: {err[:60]}", (0, 0, 255), 0.7
    return None


def _run_frame(
    cap: cv2.VideoCapture,
    cx: float,
    cy: float,
    template_half_width_px: int,
    state: CalibrationStateContainer,
    cap_lock: threading.Lock,
    stop_event: threading.Event,
) -> None:
    """Read frames, draw the guide overlay, and push annotated frames to shared state.

    Uses non-blocking lock acquisition so the sweep thread can hold the camera
    lock exclusively during frame collection without starving this thread
    indefinitely.
    """
    cx_col = int(round(cx))
    cy_row = int(round(cy))
    half_w = template_half_width_px
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
            time.sleep(0.05)
            continue

        h = frame.shape[0]
        w = frame.shape[1]
        annotated = frame.copy()
        cv2.line(annotated, (cx_col, 0), (cx_col, h - 1), (0, 255, 0), 2)
        box_x1 = max(0, cx_col - half_w)
        box_y1 = max(0, cy_row - half_w)
        box_x2 = min(w - 1, cx_col + half_w)
        box_y2 = min(h - 1, cy_row + half_w)
        cv2.rectangle(annotated, (box_x1, box_y1), (box_x2, box_y2), (0, 255, 0), 1)

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
    noise_floor_deg: float,
    camera_forward_offset_m: float,
    calibration_target_distance_m: float,
    tracking_hysteresis_enter_deg: float,
    tracking_hysteresis_exit_deg: float,
    save: bool,
) -> None:
    """Load an existing CSV, refit all models, print a summary, optionally save.

    Parameters
    ----------
    csv_path                      : Path   Path to the raw CSV file.
    sensor_config_path            : Path   Destination for sensor_config.yaml if ``save=True``.
    noise_floor_deg               : float  Passed to ``analyse_sweep``.
    camera_forward_offset_m       : float  Geometry metadata recorded in output.
    calibration_target_distance_m : float  Geometry metadata recorded in output.
    tracking_hysteresis_enter_deg : float  Runtime hysteresis enter threshold (deg).
    tracking_hysteresis_exit_deg  : float  Runtime hysteresis exit threshold (deg).
    save                          : bool   If True, write results to sensor_config.yaml.
    """
    if not csv_path.exists():
        logger.error(f"CSV not found: {csv_path}")
        sys.exit(1)

    rows = _load_csv(csv_path)
    logger.info(f"Loaded {len(rows)} rows from {csv_path}")

    model = analyse_sweep(
        rows, noise_floor_deg, camera_forward_offset_m, calibration_target_distance_m
    )

    print("\n=== Pan-Tilt Servo Curve Calibration — Replay Summary ===")
    print(f"Total samples   : {model.get('n_total_samples')}")
    print(f"Good samples    : {model.get('n_good_samples')}")

    has_error = bool(model.get("error"))
    has_no_good = int(model.get("n_good_samples", 0) or 0) == 0
    if has_error or has_no_good:
        print("Analysis failed : no good-quality samples (quality_flag==0).")
        print("=" * 54)
        if save:
            logger.error(
                "Replay analysis failed; refusing to save invalid calibration results."
            )
            sys.exit(1)
        return

    print(f"Dead-band pos   : {model.get('dead_band_pos_deg')}")
    print(f"Dead-band neg   : {model.get('dead_band_neg_deg')}")
    print(
        f"phi range       : [{model.get('phi_min_deg')}, {model.get('phi_max_deg')}] deg"
    )
    print(f"Hysteresis      : {model.get('hysteresis_mean_deg')} deg (mean)")
    lf = model.get("linear_fit", {})
    print(
        f"Linear fit      : slope={lf.get('slope'):.4f}  "
        f"intercept={lf.get('intercept'):.4f}  "
        f"MAE={lf.get('mae_deg'):.3f} deg"
    )
    pw = model.get("piecewise_linear", {})
    print(
        f"Piecewise-linear: MAE={pw.get('mae_deg'):.3f} deg  "
        f"max_err={pw.get('max_abs_error_deg'):.3f} deg"
    )
    print("=" * 54)

    if save:
        if not sensor_config_path.exists():
            logger.error(f"Sensor config not found: {sensor_config_path}")
            sys.exit(1)
        model["tracking_hysteresis_enter_deg"] = tracking_hysteresis_enter_deg
        model["tracking_hysteresis_exit_deg"] = tracking_hysteresis_exit_deg
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
            "Characterise the pan-tilt servo command→angle mapping via a "
            "commanded sweep and camera-based angle measurement."
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
        help="Override shared.noise_floor_deg from calibration config.",
    )
    return parser


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def _run_replay_mode(args: argparse.Namespace, sensor_config_path: Path) -> None:
    """Load calibration config and run offline replay analysis.

    Validates the required geometry fields from ``calibration_config.yaml``,
    then delegates to ``_run_replay``.  Exits the process on any configuration
    error so that ``main`` stays free of inline config parsing.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed CLI arguments; uses ``args.cal_config``, ``args.noise_floor``,
        ``args.replay``, and ``args.save``.
    sensor_config_path : Path
        Resolved path to ``sensor_config.yaml``, already verified to exist.
    """
    cal_config_path: Path = args.cal_config.resolve()
    if not cal_config_path.exists():
        logger.error(f"Calibration config not found: {cal_config_path}")
        sys.exit(1)
    with cal_config_path.open() as f:
        cal_cfg_replay: dict[str, Any] = yaml.safe_load(f) or {}
    pt_cfg_replay: dict[str, Any] = cal_cfg_replay.get("pan_tilt_servo", {})
    shared_cfg_replay: dict[str, Any] = cal_cfg_replay.get("shared", {})
    noise_floor: float = (
        float(args.noise_floor)
        if args.noise_floor is not None
        else float(
            shared_cfg_replay.get(
                "noise_floor_deg", pt_cfg_replay.get("noise_floor_deg", 0.5)
            )
        )
    )
    _fwd_replay = pt_cfg_replay.get("camera_forward_offset_m")
    if _fwd_replay is None:
        logger.error(
            "pan_tilt_servo.camera_forward_offset_m is missing from "
            f"{cal_config_path}. Add it before running replay."
        )
        sys.exit(1)
    _dist_replay = pt_cfg_replay.get("calibration_target_distance_m")
    if _dist_replay is None:
        logger.error(
            "pan_tilt_servo.calibration_target_distance_m is missing from "
            f"{cal_config_path}. Add it before running replay."
        )
        sys.exit(1)
    try:
        _validate_geometry(float(_fwd_replay), float(_dist_replay))
    except ValueError as exc:
        logger.error(str(exc))
        sys.exit(1)
    _run_replay(
        csv_path=args.replay.resolve(),
        sensor_config_path=sensor_config_path,
        noise_floor_deg=noise_floor,
        camera_forward_offset_m=float(_fwd_replay),
        calibration_target_distance_m=float(_dist_replay),
        tracking_hysteresis_enter_deg=float(
            pt_cfg_replay.get("tracking_hysteresis_enter_deg", 1.5)
        ),
        tracking_hysteresis_exit_deg=float(
            pt_cfg_replay.get("tracking_hysteresis_exit_deg", 3.0)
        ),
        save=args.save,
    )


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
        f"Intrinsics: cx={config.cx:.2f}px  cy={config.cy:.2f}px  fx={config.fx:.2f}px"
    )
    for i, sched in enumerate(config.pan_command_schedules_deg):
        logger.info(
            f"Schedule {i}: {len(sched)} steps, "
            f"range [{sched[0]:.1f}°, {sched[-1]:.1f}°]  "
            f"({len(sched) * 2} samples)"
        )
    total_samples = sum(len(s) * 2 for s in config.pan_command_schedules_deg)
    logger.info(f"Total sweep samples: {total_samples}")
    logger.info(
        f"Settle: {config.settle_time_s}s  Frames/step: {config.frames_to_average}  "
        f"Template half-width: {config.template_half_width_px}px"
    )

    ugv = UGVController(
        port=config.ugv_port,
        baud_rate=config.ugv_baud,
        chassis_main=config.chassis_main,
        chassis_module=config.chassis_module,
        track_width=config.track_width,
    )

    cap: cv2.VideoCapture | None = None
    frame_thread: threading.Thread | None = None
    server: ThreadingHTTPServer | None = None
    server_ref: list[ThreadingHTTPServer | None] = [None]
    state = CalibrationStateContainer()
    stop_event = threading.Event()
    cap_lock = threading.Lock()

    try:
        # -- Connect hardware and zero the pan-tilt --------------------------------
        ugv.connect()
        ugv.set_pan_tilt(0.0, config.tilt_setpoint_deg)
        logger.info(
            f"Pan-tilt commanded to (0°, {config.tilt_setpoint_deg}°). "
            f"Waiting {_SERVO_SETTLE_S}s to settle..."
        )
        time.sleep(_SERVO_SETTLE_S)

        try:
            ensure_camera_device_available(config.camera_device)
        except RuntimeError as exc:
            logger.error(str(exc))
            sys.exit(1)

        # -- Open camera -----------------------------------------------------------
        cap = cv2.VideoCapture(config.camera_device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.cam_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.cam_height)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)  # aperture priority (auto)
        if not cap.isOpened():
            logger.error(
                f"Could not open camera device {config.camera_device}. "
                "Try a different --camera-device (e.g. /dev/video0)."
            )
            sys.exit(1)

        # -- Start frame thread ----------------------------------------------------
        frame_thread = threading.Thread(
            target=_run_frame,
            args=(
                cap,
                config.cx,
                config.cy,
                config.template_half_width_px,
                state,
                cap_lock,
                stop_event,
            ),
            daemon=True,
        )
        frame_thread.start()
        logger.info("Frame thread started.")

        # -- Start HTTP server -----------------------------------------------------
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
        # -- Teardown --------------------------------------------------------------
        stop_event.set()
        if frame_thread is not None:
            frame_thread.join(timeout=2)
            if frame_thread.is_alive():
                logger.warning("Frame thread did not stop within 2 s.")
        if cap is not None:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)  # restore auto-exposure
            cap.release()
        if server is not None:
            server.server_close()
        ugv.disconnect()


if __name__ == "__main__":
    main()
