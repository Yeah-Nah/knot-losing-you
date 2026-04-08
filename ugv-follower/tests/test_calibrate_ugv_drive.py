"""Automated pytest tests for non-hardware logic in calibrate_ugv_drive.

All tests run without any physical hardware.  They import pure functions and
CSV/YAML helpers directly from the calibration script.
"""

from __future__ import annotations

import argparse
import math
import os
import tempfile
import threading
import time
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
import yaml

from tools.calibration.calibrate_ugv_drive import (
    CalibrationStateContainer,
    DriveCalConfig,
    _build_dead_band_row,
    _build_gain_row,
    _load_config,
    _load_csv,
    _run_sweep,
    _validate_geometry,
    _write_csv,
    _write_results_atomic,
    analyse_runs,
    correct_for_camera_offset,
    estimate_dead_band,
    fit_turn_rate_gain,
    quality_flag,
)
from ugv_follower.utils.fisheye_utils import (
    load_fisheye_intrinsics,
    pixel_to_bearing_deg,
    pixel_to_normalised,
)


# ---------------------------------------------------------------------------
# Fixtures and helpers
# ---------------------------------------------------------------------------


def _make_K_D_zero_distortion() -> tuple[np.ndarray, np.ndarray]:
    """Return a simple camera matrix and zero distortion coefficients.

    With D=zeros, cv2.fisheye.undistortPoints behaves like the pinhole model:
    x_n = (u - cx) / fx, y_n = (v - cy) / fy.
    """
    K = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])
    D = np.zeros((4, 1), dtype=np.float64)
    return K, D


def _make_minimal_sensor_cfg() -> dict[str, Any]:
    """Minimal sensor_config.yaml content for config loading tests."""
    return {
        "waveshare_rgb": {
            "model": "fisheye",
            "camera_matrix": [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]],
            "dist_coeffs": [0.0, 0.0, 0.0, 0.0],
            "resolution": [640, 480],
        },
        "ugv": {
            "port": "/dev/ttyAMA0",
            "baud_rate": 115200,
            "chassis_main": 2,
            "chassis_module": 2,
            "track_width": 0.3,
        },
    }


def _make_minimal_cal_cfg(
    camera_offset_m: float = 0.10,
    target_distance_m: float = 2.0,
) -> dict[str, Any]:
    """Minimal calibration_config.yaml content for config loading tests."""
    return {
        "ugv_drive": {
            "omega_commands_rad_s": [0.5, 1.0],
            "command_duration_s": 1.0,
            "settle_time_s": 1.0,
            "dead_band_omega_steps_rad_s": [0.2, 0.1],
            "dead_band_duration_s": 1.0,
            "dead_band_settle_s": 1.0,
            "frames_to_average": 3,
            "noise_floor_deg": 1.5,
            "camera_offset_m": camera_offset_m,
            "target_distance_m": target_distance_m,
            "template_half_width_px": 40,
            "min_match_score": 0.6,
            "camera_device": "/dev/video0",
            "calibration_surface": "test floor",
        }
    }


def _make_args(tmp_path: Path) -> argparse.Namespace:
    ns = argparse.Namespace()
    ns.camera_device = None
    ns.noise_floor = None
    sensor_path = tmp_path / "sensor.yaml"
    sensor_path.write_text("{}")
    ns.sensor_config = sensor_path
    return ns


def _make_drive_cal_config(
    omega_commands: tuple[float, ...] = (0.5, 1.0),
    dead_band_steps: tuple[float, ...] = (0.2, 0.1),
    settle_time_s: float = 0.0,
    dead_band_settle_s: float = 0.0,
    command_duration_s: float = 0.0,
    dead_band_duration_s: float = 0.0,
    noise_floor_deg: float = 1.5,
    camera_offset_m: float = 0.0,
    target_distance_m: float = 2.0,
) -> DriveCalConfig:
    """Build a minimal DriveCalConfig suitable for unit tests."""
    K, D = _make_K_D_zero_distortion()
    return DriveCalConfig(
        K=K,
        D=D,
        frame_width=640,
        frame_height=480,
        cx=320.0,
        ugv_port="/dev/null",
        ugv_baud_rate=115200,
        ugv_chassis_main=2,
        ugv_chassis_module=2,
        ugv_track_width_nom=0.3,
        omega_commands_rad_s=omega_commands,
        command_duration_s=command_duration_s,
        settle_time_s=settle_time_s,
        dead_band_omega_steps_rad_s=dead_band_steps,
        dead_band_duration_s=dead_band_duration_s,
        dead_band_settle_s=dead_band_settle_s,
        frames_to_average=1,
        noise_floor_deg=noise_floor_deg,
        camera_offset_m=camera_offset_m,
        target_distance_m=target_distance_m,
        template_half_width_px=20,
        min_match_score=0.6,
        camera_device="/dev/video0",
        calibration_surface="test floor",
        sensor_config_path=Path("/tmp/sensor.yaml"),
    )


def _make_gain_rows(
    omegas: list[float] = None,
    k_actual: float = 1.0,
    duration_s: float = 2.0,
    quality: int = 0,
) -> list[dict[str, Any]]:
    """Build synthetic gain_sweep rows for analysis tests."""
    if omegas is None:
        omegas = [0.5, 1.0]
    rows = []
    for omega in omegas:
        for direction, sign in [("ccw", 1.0), ("cw", -1.0)]:
            expected = math.degrees(sign * omega * duration_s)
            corrected = k_actual * expected
            rows.append({
                "run_type": "gain_sweep",
                "omega_commanded_rad_s": omega,
                "direction": direction,
                "command_duration_s": duration_s,
                "bearing_before_deg": 0.0,
                "bearing_after_deg": corrected,
                "delta_bearing_deg": corrected,
                "corrected_delta_deg": corrected,
                "expected_angle_deg": expected,
                "moved": None,
                "match_score_before": 0.9,
                "match_score_after": 0.9,
                "quality_flag": quality,
                "timestamp_s": 0.0,
            })
    return rows


def _make_dead_band_rows(
    omegas: list[float] = None,
    moved_flags: list[bool] = None,
) -> list[dict[str, Any]]:
    """Build synthetic dead_band rows for analysis tests."""
    if omegas is None:
        omegas = [0.3, 0.2, 0.1]
    if moved_flags is None:
        moved_flags = [True, True, False]
    rows = []
    for omega, moved in zip(omegas, moved_flags):
        for direction in ("ccw", "cw"):
            rows.append({
                "run_type": "dead_band",
                "omega_commanded_rad_s": omega,
                "direction": direction,
                "command_duration_s": 1.5,
                "bearing_before_deg": 0.0,
                "bearing_after_deg": 5.0 if moved else 0.0,
                "delta_bearing_deg": 5.0 if moved else 0.0,
                "corrected_delta_deg": None,
                "expected_angle_deg": None,
                "moved": moved,
                "match_score_before": 0.9,
                "match_score_after": 0.9,
                "quality_flag": 0,
                "timestamp_s": 0.0,
            })
    return rows


# ---------------------------------------------------------------------------
# fisheye_utils — pixel_to_normalised
# ---------------------------------------------------------------------------


def test_pixel_to_normalised_centre_is_origin() -> None:
    """Principal point (cx, cy) → normalised coordinates (0, 0)."""
    K, D = _make_K_D_zero_distortion()
    cx, cy = float(K[0, 2]), float(K[1, 2])
    x_n, y_n = pixel_to_normalised(cx, cy, K, D)
    assert abs(x_n) < 1e-6
    assert abs(y_n) < 1e-6


def test_pixel_to_normalised_right_of_centre() -> None:
    """Pixel to the right of cx → positive x_n."""
    K, D = _make_K_D_zero_distortion()
    cx, cy = float(K[0, 2]), float(K[1, 2])
    x_n, _y_n = pixel_to_normalised(cx + 100.0, cy, K, D)
    assert x_n > 0.0


def test_pixel_to_normalised_known_value_zero_distortion() -> None:
    """With D=zeros, undistortion matches the pinhole model: x_n = (u - cx) / fx."""
    K, D = _make_K_D_zero_distortion()
    cx, cy = float(K[0, 2]), float(K[1, 2])
    fx = float(K[0, 0])
    offset = 200.0
    x_n, _y_n = pixel_to_normalised(cx + offset, cy, K, D)
    assert x_n == pytest.approx(offset / fx, rel=1e-4)


# ---------------------------------------------------------------------------
# fisheye_utils — pixel_to_bearing_deg
# ---------------------------------------------------------------------------


def test_pixel_to_bearing_deg_centre_is_zero() -> None:
    """Principal point → bearing of exactly 0°."""
    K, D = _make_K_D_zero_distortion()
    cx, cy = float(K[0, 2]), float(K[1, 2])
    assert pixel_to_bearing_deg(cx, cy, K, D) == pytest.approx(0.0, abs=1e-6)


def test_pixel_to_bearing_deg_right_of_centre_is_positive() -> None:
    """Target to the right of optical axis → positive bearing."""
    K, D = _make_K_D_zero_distortion()
    cx, cy = float(K[0, 2]), float(K[1, 2])
    bearing = pixel_to_bearing_deg(cx + 50.0, cy, K, D)
    assert bearing > 0.0


def test_pixel_to_bearing_deg_left_of_centre_is_negative() -> None:
    """Target to the left of optical axis → negative bearing."""
    K, D = _make_K_D_zero_distortion()
    cx, cy = float(K[0, 2]), float(K[1, 2])
    bearing = pixel_to_bearing_deg(cx - 50.0, cy, K, D)
    assert bearing < 0.0


def test_pixel_to_bearing_deg_known_value_zero_distortion() -> None:
    """With D=zeros and x_n=1 (u = cx + fx), bearing should be atan(1) = 45°."""
    K, D = _make_K_D_zero_distortion()
    cx, cy = float(K[0, 2]), float(K[1, 2])
    fx = float(K[0, 0])
    bearing = pixel_to_bearing_deg(cx + fx, cy, K, D)
    assert bearing == pytest.approx(45.0, rel=1e-4)


def test_pixel_to_bearing_deg_antisymmetric() -> None:
    """Bearing is antisymmetric about cx: bearing(cx+d) == -bearing(cx-d)."""
    K, D = _make_K_D_zero_distortion()
    cx, cy = float(K[0, 2]), float(K[1, 2])
    b_right = pixel_to_bearing_deg(cx + 150.0, cy, K, D)
    b_left = pixel_to_bearing_deg(cx - 150.0, cy, K, D)
    assert b_right == pytest.approx(-b_left, rel=1e-4)


# ---------------------------------------------------------------------------
# fisheye_utils — load_fisheye_intrinsics
# ---------------------------------------------------------------------------


def test_load_fisheye_intrinsics_shapes() -> None:
    """Loaded K is (3, 3) and D is (4, 1)."""
    cfg = _make_minimal_sensor_cfg()
    K, D = load_fisheye_intrinsics(cfg)
    assert K.shape == (3, 3)
    assert D.shape == (4, 1)


def test_load_fisheye_intrinsics_missing_matrix_raises() -> None:
    """Missing camera_matrix → ValueError."""
    with pytest.raises(ValueError, match="camera_matrix is null"):
        load_fisheye_intrinsics({"waveshare_rgb": {"dist_coeffs": [0, 0, 0, 0]}})


def test_load_fisheye_intrinsics_missing_dist_raises() -> None:
    """Missing dist_coeffs → ValueError."""
    with pytest.raises(ValueError, match="dist_coeffs is null"):
        load_fisheye_intrinsics({
            "waveshare_rgb": {
                "camera_matrix": [[500, 0, 320], [0, 500, 240], [0, 0, 1]]
            }
        })


# ---------------------------------------------------------------------------
# correct_for_camera_offset
# ---------------------------------------------------------------------------


def test_correct_for_camera_offset_zero_offset_is_identity() -> None:
    """d=0 → corrected delta equals input delta."""
    assert correct_for_camera_offset(30.0, d_m=0.0, D_m=2.0) == pytest.approx(30.0)
    assert correct_for_camera_offset(-30.0, d_m=0.0, D_m=2.0) == pytest.approx(-30.0)
    assert correct_for_camera_offset(0.0, d_m=0.0, D_m=2.0) == pytest.approx(0.0)


def test_correct_for_camera_offset_known_value() -> None:
    """Spot-check against manually computed value."""
    delta = 30.0
    d, D = 0.15, 2.0
    expected = delta - math.degrees(math.asin((d / D) * math.sin(math.radians(delta))))
    assert correct_for_camera_offset(delta, d, D) == pytest.approx(expected, rel=1e-6)


def test_correct_for_camera_offset_antisymmetric() -> None:
    """Correction is antisymmetric: correct(-delta) == -correct(+delta)."""
    pos = correct_for_camera_offset(25.0, d_m=0.15, D_m=2.0)
    neg = correct_for_camera_offset(-25.0, d_m=0.15, D_m=2.0)
    assert neg == pytest.approx(-pos, rel=1e-6)


def test_correct_for_camera_offset_compression() -> None:
    """Corrected bearing change is strictly smaller in magnitude than the raw value."""
    corrected = correct_for_camera_offset(40.0, d_m=0.15, D_m=2.0)
    assert abs(corrected) < 40.0


def test_correct_for_camera_offset_domain_violation() -> None:
    """Arcsin domain violation (|d/D·sin(delta)| >= 1) → ValueError."""
    with pytest.raises(ValueError, match="Trig domain violation"):
        correct_for_camera_offset(90.0, d_m=2.0, D_m=2.0)


# ---------------------------------------------------------------------------
# _validate_geometry
# ---------------------------------------------------------------------------


def test_validate_geometry_valid() -> None:
    """Valid inputs must not raise."""
    _validate_geometry(0.10, 2.0)
    _validate_geometry(0.0, 0.01)


def test_validate_geometry_non_positive_distance() -> None:
    """target_distance_m <= 0 → ValueError."""
    with pytest.raises(ValueError, match="target_distance_m must be positive"):
        _validate_geometry(0.1, 0.0)
    with pytest.raises(ValueError, match="target_distance_m must be positive"):
        _validate_geometry(0.1, -1.0)


def test_validate_geometry_negative_offset() -> None:
    """camera_offset_m < 0 → ValueError."""
    with pytest.raises(ValueError, match="camera_offset_m must be non-negative"):
        _validate_geometry(-0.01, 2.0)


def test_validate_geometry_offset_ge_distance() -> None:
    """camera_offset_m >= target_distance_m → ValueError."""
    with pytest.raises(ValueError):
        _validate_geometry(2.0, 2.0)
    with pytest.raises(ValueError):
        _validate_geometry(3.0, 2.0)


# ---------------------------------------------------------------------------
# quality_flag
# ---------------------------------------------------------------------------


def test_quality_flag_good() -> None:
    """High match score, centroid well within frame → flag 0."""
    assert quality_flag(0.9, 320.0, 640, min_match_score=0.65) == 0


def test_quality_flag_low_match() -> None:
    """Match score below threshold → flag 1."""
    assert quality_flag(0.5, 320.0, 640, min_match_score=0.65) == 1


def test_quality_flag_near_edge() -> None:
    """Centroid within edge_margin_px of frame boundary → flag 2."""
    assert quality_flag(0.9, 5.0, 640, min_match_score=0.65) == 2
    assert quality_flag(0.9, 635.0, 640, min_match_score=0.65) == 2


def test_quality_flag_both() -> None:
    """Low score AND near edge → flag 3."""
    assert quality_flag(0.3, 5.0, 640, min_match_score=0.65) == 3


def test_quality_flag_bitor_combines_two_measurements() -> None:
    """Bitwise OR of before/after flags: low_match | near_edge = 3."""
    flag_before = quality_flag(0.4, 320.0, 640, min_match_score=0.65)   # 1 (low_match)
    flag_after = quality_flag(0.9, 5.0, 640, min_match_score=0.65)       # 2 (near_edge)
    assert (flag_before | flag_after) == 3


# ---------------------------------------------------------------------------
# fit_turn_rate_gain
# ---------------------------------------------------------------------------


def test_fit_turn_rate_gain_identity() -> None:
    """delta == expected → k = 1.0, MAE = 0."""
    expected = [57.3, 114.6, -57.3, -114.6]
    result = fit_turn_rate_gain(expected, expected)
    assert result["slope"] == pytest.approx(1.0, rel=1e-6)
    assert result["mae_deg"] == pytest.approx(0.0, abs=1e-6)
    assert result["max_abs_error_deg"] == pytest.approx(0.0, abs=1e-6)


def test_fit_turn_rate_gain_over_turn() -> None:
    """Rover over-turns by 20% → k = 1.2."""
    expected = [57.3, 114.6, -57.3, -114.6]
    actual = [v * 1.2 for v in expected]
    result = fit_turn_rate_gain(actual, expected)
    assert result["slope"] == pytest.approx(1.2, rel=1e-5)


def test_fit_turn_rate_gain_too_few_samples() -> None:
    """Fewer than 2 samples → ValueError."""
    with pytest.raises(ValueError, match="at least 2"):
        fit_turn_rate_gain([1.0], [1.0])


def test_fit_turn_rate_gain_residuals_nonzero() -> None:
    """Noisy data produces non-zero MAE."""
    expected = [57.3, 114.6, -57.3, -114.6]
    noisy = [62.0, 120.0, -55.0, -110.0]   # not perfectly proportional
    result = fit_turn_rate_gain(noisy, expected)
    assert result["mae_deg"] > 0.0


# ---------------------------------------------------------------------------
# estimate_dead_band
# ---------------------------------------------------------------------------


def test_estimate_dead_band_all_moved_returns_none() -> None:
    """All omegas produce rotation → no dead-band detected → None."""
    omegas = [0.30, 0.25, 0.20, 0.15, 0.10]
    moved = [True, True, True, True, True]
    assert estimate_dead_band(omegas, moved) is None


def test_estimate_dead_band_clear_threshold_midpoint() -> None:
    """Clear crossover at 0.20/0.25 → midpoint = 0.225."""
    omegas = [0.30, 0.25, 0.20, 0.15, 0.10]
    moved = [True, True, False, False, False]
    result = estimate_dead_band(omegas, moved)
    assert result == pytest.approx(0.225, rel=1e-6)


def test_estimate_dead_band_all_stalled_returns_max() -> None:
    """All omegas stall → conservative estimate = max tested omega."""
    omegas = [0.30, 0.25, 0.20, 0.15]
    moved = [False, False, False, False]
    assert estimate_dead_band(omegas, moved) == pytest.approx(0.30)


def test_estimate_dead_band_empty_returns_none() -> None:
    """Empty input → None."""
    assert estimate_dead_band([], []) is None


# ---------------------------------------------------------------------------
# analyse_runs
# ---------------------------------------------------------------------------


def test_analyse_runs_required_keys() -> None:
    """Result contains all keys needed for sensor_config.yaml."""
    config = _make_drive_cal_config()
    rows = _make_gain_rows() + _make_dead_band_rows()
    result = analyse_runs(rows, config)
    required = {
        "calibration_surface",
        "effective_track_width_m",
        "turn_rate_gain",
        "turn_rate_gain_pos",
        "turn_rate_gain_neg",
        "angular_dead_band_rad_s",
        "n_samples",
        "fit_mae_deg",
        "n_total_samples",
    }
    assert required.issubset(result.keys())


def test_analyse_runs_excludes_bad_quality_rows() -> None:
    """Gain rows with quality_flag != 0 are excluded from the fit."""
    config = _make_drive_cal_config()
    good_rows = _make_gain_rows(omegas=[1.0], k_actual=1.0, quality=0)
    bad_rows = _make_gain_rows(omegas=[0.5], k_actual=99.0, quality=1)   # should be ignored
    result = analyse_runs(good_rows + bad_rows, config)
    # Only 2 good gain rows (1 omega × CCW + CW), not the 2 bad ones
    assert result["n_samples"] == 2


def test_analyse_runs_identity_gain() -> None:
    """Synthetic data with k=1 → turn_rate_gain ≈ 1.0."""
    config = _make_drive_cal_config()
    rows = _make_gain_rows(k_actual=1.0)
    result = analyse_runs(rows, config)
    assert result["turn_rate_gain"] == pytest.approx(1.0, rel=1e-4)
    assert result["effective_track_width_m"] == pytest.approx(config.ugv_track_width_nom, rel=1e-4)


def test_analyse_runs_over_turn_gain() -> None:
    """Synthetic data with k=1.2 → W_eff = W_nom / 1.2."""
    config = _make_drive_cal_config()
    rows = _make_gain_rows(k_actual=1.2)
    result = analyse_runs(rows, config)
    assert result["turn_rate_gain"] == pytest.approx(1.2, rel=1e-3)
    expected_w = config.ugv_track_width_nom / 1.2
    assert result["effective_track_width_m"] == pytest.approx(expected_w, rel=1e-3)


def test_analyse_runs_symmetric_gain_no_split() -> None:
    """Symmetric CCW/CW → turn_rate_gain_pos and turn_rate_gain_neg are None."""
    config = _make_drive_cal_config()
    rows = _make_gain_rows(k_actual=1.0)
    result = analyse_runs(rows, config)
    assert result["turn_rate_gain_pos"] is None
    assert result["turn_rate_gain_neg"] is None


def test_analyse_runs_insufficient_good_samples() -> None:
    """Fewer than 2 good gain_sweep rows → error key present."""
    config = _make_drive_cal_config()
    rows = _make_gain_rows(quality=1)   # all bad quality
    result = analyse_runs(rows, config)
    assert "error" in result


def test_analyse_runs_dead_band_detected() -> None:
    """Clear dead-band in synthetic data → angular_dead_band_rad_s is set."""
    config = _make_drive_cal_config(noise_floor_deg=1.5)
    rows = _make_gain_rows() + _make_dead_band_rows(
        omegas=[0.30, 0.25, 0.20, 0.15],
        moved_flags=[True, True, False, False],
    )
    result = analyse_runs(rows, config)
    assert result["angular_dead_band_rad_s"] is not None
    assert result["angular_dead_band_rad_s"] == pytest.approx(0.225, rel=1e-4)


def test_analyse_runs_no_dead_band_rows() -> None:
    """No dead_band rows → angular_dead_band_rad_s is None."""
    config = _make_drive_cal_config()
    rows = _make_gain_rows()
    result = analyse_runs(rows, config)
    assert result["angular_dead_band_rad_s"] is None


# ---------------------------------------------------------------------------
# CSV round-trip
# ---------------------------------------------------------------------------


def _make_sample_rows() -> list[dict[str, Any]]:
    t0 = time.monotonic()
    return [
        _build_gain_row(
            omega=0.5, direction="ccw", duration_s=2.0,
            b_before=1.23, b_after=58.6, corrected_delta=55.4, expected_angle=57.3,
            score_before=0.91, score_after=0.88, qflag=0, t0=t0,
        ),
        _build_gain_row(
            omega=0.5, direction="cw", duration_s=2.0,
            b_before=58.6, b_after=1.3, corrected_delta=-55.2, expected_angle=-57.3,
            score_before=0.89, score_after=0.92, qflag=0, t0=t0,
        ),
        _build_dead_band_row(
            omega=0.2, direction="ccw", duration_s=1.5,
            b_before=0.0, b_after=3.5, moved=True,
            score_before=0.87, score_after=0.85, qflag=0, t0=t0,
        ),
        _build_dead_band_row(
            omega=0.1, direction="cw", duration_s=1.5,
            b_before=0.0, b_after=0.3, moved=False,
            score_before=0.86, score_after=0.84, qflag=0, t0=t0,
        ),
    ]


def test_csv_round_trip_types(tmp_path: Path) -> None:
    """Write → read → verify numeric, string, and bool types are preserved."""
    rows = _make_sample_rows()
    csv_path = tmp_path / "test.csv"
    _write_csv(csv_path, rows)
    loaded = _load_csv(csv_path)

    assert len(loaded) == 4

    gain_row = loaded[0]
    assert isinstance(gain_row["timestamp_s"], float)
    assert isinstance(gain_row["run_type"], str)
    assert gain_row["run_type"] == "gain_sweep"
    assert isinstance(gain_row["quality_flag"], int)
    assert isinstance(gain_row["corrected_delta_deg"], float)
    assert gain_row["moved"] is None   # empty for gain_sweep

    dead_row = loaded[2]
    assert dead_row["run_type"] == "dead_band"
    assert dead_row["moved"] is True
    assert dead_row["corrected_delta_deg"] is None

    dead_row_stalled = loaded[3]
    assert dead_row_stalled["moved"] is False


def test_csv_round_trip_values_preserved(tmp_path: Path) -> None:
    """Bearing and score values survive the CSV round-trip without precision loss."""
    rows = _make_sample_rows()
    csv_path = tmp_path / "test.csv"
    _write_csv(csv_path, rows)
    loaded = _load_csv(csv_path)

    assert loaded[0]["bearing_before_deg"] == pytest.approx(1.23, rel=1e-4)
    assert loaded[0]["match_score_before"] == pytest.approx(0.91, rel=1e-4)
    assert loaded[0]["corrected_delta_deg"] == pytest.approx(55.4, rel=1e-4)


# ---------------------------------------------------------------------------
# Atomic YAML write
# ---------------------------------------------------------------------------


def test_write_results_atomic_preserves_other_keys(tmp_path: Path) -> None:
    """Writing ugv_drive must not remove other top-level keys."""
    sensor_cfg = tmp_path / "sensor_config.yaml"
    existing = {"camera": {"fps": 30}, "lidar": {"port": "/dev/ttyUSB0"}}
    with sensor_cfg.open("w") as f:
        yaml.dump(existing, f)

    result_dict = {"turn_rate_gain": 0.95, "calibrated_at": "2026-04-08T10:00:00"}
    _write_results_atomic(sensor_cfg, result_dict)

    with sensor_cfg.open() as f:
        loaded = yaml.safe_load(f)

    assert loaded["camera"]["fps"] == 30
    assert loaded["lidar"]["port"] == "/dev/ttyUSB0"
    assert loaded["ugv_drive"]["turn_rate_gain"] == pytest.approx(0.95)


def test_write_results_atomic_replaces_existing_ugv_drive(tmp_path: Path) -> None:
    """Calling _write_results_atomic twice replaces the first ugv_drive section."""
    sensor_cfg = tmp_path / "sensor_config.yaml"
    with sensor_cfg.open("w") as f:
        yaml.dump({"ugv_drive": {"turn_rate_gain": 0.80}}, f)

    _write_results_atomic(sensor_cfg, {"turn_rate_gain": 0.95})

    with sensor_cfg.open() as f:
        loaded = yaml.safe_load(f)
    assert loaded["ugv_drive"]["turn_rate_gain"] == pytest.approx(0.95)


# ---------------------------------------------------------------------------
# Config loading — _load_config
# ---------------------------------------------------------------------------


def test_load_config_missing_ugv_drive_section_raises(tmp_path: Path) -> None:
    """Missing ugv_drive section in calibration config → ValueError."""
    with pytest.raises(ValueError, match="ugv_drive section is missing"):
        _load_config(
            _make_minimal_sensor_cfg(),
            {},   # no ugv_drive key
            _make_args(tmp_path),
        )


def test_load_config_geometry_validated(tmp_path: Path) -> None:
    """camera_offset_m >= target_distance_m → ValueError from geometry validation."""
    with pytest.raises(ValueError):
        _load_config(
            _make_minimal_sensor_cfg(),
            _make_minimal_cal_cfg(camera_offset_m=3.0, target_distance_m=2.0),
            _make_args(tmp_path),
        )


def test_load_config_noise_floor_cli_override(tmp_path: Path) -> None:
    """--noise-floor CLI arg overrides config value."""
    args = _make_args(tmp_path)
    args.noise_floor = 3.0
    cfg = _load_config(_make_minimal_sensor_cfg(), _make_minimal_cal_cfg(), args)
    assert cfg.noise_floor_deg == pytest.approx(3.0)


def test_load_config_loads_omega_commands(tmp_path: Path) -> None:
    """omega_commands_rad_s from config is loaded as a tuple of floats."""
    cfg = _load_config(
        _make_minimal_sensor_cfg(), _make_minimal_cal_cfg(), _make_args(tmp_path)
    )
    assert cfg.omega_commands_rad_s == (0.5, 1.0)


# ---------------------------------------------------------------------------
# _run_sweep — row count and UGV call count
# ---------------------------------------------------------------------------


@patch(
    "tools.calibration.calibrate_ugv_drive._capture_bearing_measurement",
    return_value=(5.0, 320.0, 0.90),
)
@patch("tools.calibration.calibrate_ugv_drive.time.sleep")
def test_run_sweep_row_count(mock_sleep: MagicMock, mock_bearing: MagicMock) -> None:
    """2 omega commands + 2 dead-band steps → 4 gain rows + 4 dead-band rows = 8 total."""
    config = _make_drive_cal_config(
        omega_commands=(0.5, 1.0),
        dead_band_steps=(0.2, 0.1),
    )
    state = CalibrationStateContainer()
    state.try_start_sweep()
    ugv_mock = MagicMock()

    _run_sweep(
        MagicMock(),   # cap — not used because _capture_bearing_measurement is mocked
        ugv_mock,
        config,
        np.zeros((40, 40, 3), dtype=np.uint8),
        state,
        threading.Lock(),
        threading.Event(),
        time.monotonic(),
    )

    rows = state.get_sweep_rows()
    assert rows is not None
    assert len(rows) == 8

    gain_rows = [r for r in rows if r["run_type"] == "gain_sweep"]
    dead_rows = [r for r in rows if r["run_type"] == "dead_band"]
    assert len(gain_rows) == 4   # 2 omegas × CCW + CW
    assert len(dead_rows) == 4   # 2 dead-band steps × CCW + CW


@patch(
    "tools.calibration.calibrate_ugv_drive._capture_bearing_measurement",
    return_value=(5.0, 320.0, 0.90),
)
@patch("tools.calibration.calibrate_ugv_drive.time.sleep")
def test_run_sweep_ugv_move_and_stop_calls(
    mock_sleep: MagicMock, mock_bearing: MagicMock
) -> None:
    """Each rotation run issues exactly one ugv.move and one ugv.stop."""
    config = _make_drive_cal_config(
        omega_commands=(0.5,),
        dead_band_steps=(0.2,),
    )
    state = CalibrationStateContainer()
    state.try_start_sweep()
    ugv_mock = MagicMock()

    _run_sweep(
        MagicMock(), ugv_mock, config,
        np.zeros((40, 40, 3), dtype=np.uint8),
        state, threading.Lock(), threading.Event(), time.monotonic(),
    )

    # 1 omega: 1 CCW + 1 CW = 2 gain runs
    # 1 dead_band step: 1 CCW + 1 CW = 2 dead-band runs
    # Total: 4 move calls, 4 stop calls
    assert ugv_mock.move.call_count == 4
    assert ugv_mock.stop.call_count == 4


@patch(
    "tools.calibration.calibrate_ugv_drive._capture_bearing_measurement",
    return_value=(5.0, 320.0, 0.90),
)
@patch("tools.calibration.calibrate_ugv_drive.time.sleep")
def test_run_sweep_ccw_and_cw_signs(
    mock_sleep: MagicMock, mock_bearing: MagicMock
) -> None:
    """CCW run uses +omega; CW run uses -omega."""
    config = _make_drive_cal_config(omega_commands=(0.7,), dead_band_steps=())
    state = CalibrationStateContainer()
    state.try_start_sweep()
    ugv_mock = MagicMock()

    _run_sweep(
        MagicMock(), ugv_mock, config,
        np.zeros((40, 40, 3), dtype=np.uint8),
        state, threading.Lock(), threading.Event(), time.monotonic(),
    )

    move_args = [call.args for call in ugv_mock.move.call_args_list]
    # CCW: move(0.0, +0.7); CW: move(0.0, -0.7)
    assert move_args[0] == (0.0, pytest.approx(0.7))
    assert move_args[1] == (0.0, pytest.approx(-0.7))


# ---------------------------------------------------------------------------
# Replay-like integration test
# ---------------------------------------------------------------------------


def test_replay_analyse_runs_from_csv(tmp_path: Path) -> None:
    """Write rows to CSV, load them back, run analyse_runs, check key output."""
    config = _make_drive_cal_config()
    rows = _make_gain_rows(k_actual=1.05) + _make_dead_band_rows(
        omegas=[0.20, 0.10],
        moved_flags=[True, False],
    )
    csv_path = tmp_path / "run.csv"
    _write_csv(csv_path, rows)
    loaded = _load_csv(csv_path)

    result = analyse_runs(loaded, config)
    assert result.get("turn_rate_gain") is not None
    assert result["turn_rate_gain"] == pytest.approx(1.05, rel=1e-3)
    assert result["angular_dead_band_rad_s"] is not None
