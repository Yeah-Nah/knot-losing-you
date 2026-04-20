"""Automated pytest tests for non-hardware logic in calibrate_ugv_drive.

All tests run without any physical hardware.  They import pure functions and
CSV/YAML helpers directly from the calibration script.
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock, patch

import cv2
import numpy as np
import pytest
import yaml

from tools.calibration.calibrate_ugv_drive import (
    CalibrationOrchestrator,
    CalibrationState,
    CalibrationStateContainer,
    DriveCalConfig,
    SweepStatus,
    _SweepCancelled,
    _bearing_from_distorted_click,
    _build_dead_band_row,
    _build_gain_row,
    _check_cancel,
    _collect_click_bearing,
    _get_status_text,
    _load_config,
    _load_csv,
    _run_sweep,
    _sign_consistent,
    _try_reopen_capture,
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
    pixel_to_bearing_deg_pinhole,
    pixel_to_normalised,
    undistort_frame,
)


# ---------------------------------------------------------------------------
# Fixtures and helpers
# ---------------------------------------------------------------------------


def _make_K_D_zero_distortion() -> tuple[np.ndarray, np.ndarray]:
    """Return a simple camera matrix and zero distortion coefficients.

    With D=zeros, cv2.fisheye.undistortPoints uses the equidistant model:
    theta = (u - cx) / fx, x_n = tan(theta) — not the pinhole x_n = (u-cx)/fx.
    """
    K = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])
    D = np.zeros((4, 1), dtype=np.float64)
    return K, D


def _make_minimal_sensor_cfg() -> dict[str, Any]:
    """Minimal sensor_config.yaml content for config loading tests."""
    return {
        "waveshare_rgb": {
            "model": "fisheye",
            "camera_matrix": [
                [500.0, 0.0, 320.0],
                [0.0, 500.0, 240.0],
                [0.0, 0.0, 1.0],
            ],
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
        "shared": {
            "command_duration_s": 1.0,
            "settle_time_s": 1.0,
            "noise_floor_deg": 1.5,
            "template_half_width_px": 40,
            "camera_device": "/dev/video0",
        },
        "ugv_drive": {
            "omega_commands_rad_s": [0.5, 1.0],
            "dead_band_omega_steps_rad_s": [0.2, 0.1],
            "camera_offset_m": camera_offset_m,
            "target_distance_m": target_distance_m,
            "calibration_surface": "test floor",
        },
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
    command_duration_s: float = 0.0,
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
        cy=240.0,
        ugv_port="/dev/null",
        ugv_baud_rate=115200,
        ugv_chassis_main=2,
        ugv_chassis_module=2,
        ugv_track_width_nom=0.3,
        omega_commands_rad_s=omega_commands,
        command_duration_s=command_duration_s,
        settle_time_s=settle_time_s,
        dead_band_omega_steps_rad_s=dead_band_steps,
        noise_floor_deg=noise_floor_deg,
        camera_offset_m=camera_offset_m,
        target_distance_m=target_distance_m,
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
            rows.append(
                {
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
                }
            )
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
            rows.append(
                {
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
                }
            )
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
    """With D=zeros, equidistant model gives x_n = tan((u - cx) / fx)."""
    K, D = _make_K_D_zero_distortion()
    cx, cy = float(K[0, 2]), float(K[1, 2])
    fx = float(K[0, 0])
    offset = 200.0
    x_n, _y_n = pixel_to_normalised(cx + offset, cy, K, D)
    assert x_n == pytest.approx(math.tan(offset / fx), rel=1e-4)


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
    """With D=zeros and u = cx + fx, bearing = atan(tan(1 rad)) = 1 rad = 57.3°."""
    K, D = _make_K_D_zero_distortion()
    cx, cy = float(K[0, 2]), float(K[1, 2])
    fx = float(K[0, 0])
    bearing = pixel_to_bearing_deg(cx + fx, cy, K, D)
    assert bearing == pytest.approx(math.degrees(1.0), rel=1e-4)


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
        load_fisheye_intrinsics(
            {
                "waveshare_rgb": {
                    "camera_matrix": [[500, 0, 320], [0, 500, 240], [0, 0, 1]]
                }
            }
        )


# ---------------------------------------------------------------------------
# fisheye_utils — undistort_frame and pixel_to_bearing_deg_pinhole
# ---------------------------------------------------------------------------


def test_undistort_frame_returns_same_shape() -> None:
    """Output frame has the same shape as the input."""
    K, D = _make_K_D_zero_distortion()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    assert undistort_frame(frame, K, D).shape == frame.shape


def test_pixel_to_bearing_deg_pinhole_centre_is_zero() -> None:
    """Principal point u=cx → bearing of exactly 0°."""
    K, _ = _make_K_D_zero_distortion()
    cx, fx = float(K[0, 2]), float(K[0, 0])
    assert pixel_to_bearing_deg_pinhole(cx, cx, fx) == pytest.approx(0.0, abs=1e-9)


def test_pixel_to_bearing_deg_pinhole_known_value() -> None:
    """u = cx + fx → x_n = 1 → bearing = atan(1) = 45°."""
    K, _ = _make_K_D_zero_distortion()
    cx, fx = float(K[0, 2]), float(K[0, 0])
    assert pixel_to_bearing_deg_pinhole(cx + fx, cx, fx) == pytest.approx(
        45.0, rel=1e-6
    )


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
    """Centroid well within frame → flag 0."""
    assert quality_flag(320.0, 640) == 0


def test_quality_flag_near_edge_left() -> None:
    """Centroid within edge_margin_px of left boundary → flag 2."""
    assert quality_flag(5.0, 640) == 2


def test_quality_flag_near_edge_right() -> None:
    """Centroid within edge_margin_px of right boundary → flag 2."""
    assert quality_flag(635.0, 640) == 2


def test_quality_flag_bitor_combines_two_measurements() -> None:
    """Bitwise OR of before/after flags: good | near_edge = 2."""
    flag_before = quality_flag(320.0, 640)  # 0 (good)
    flag_after = quality_flag(5.0, 640)  # 2 (near_edge)
    assert (flag_before | flag_after) == 2


# ---------------------------------------------------------------------------
# _sign_consistent
# ---------------------------------------------------------------------------


def test_sign_consistent_matching_signs() -> None:
    """Both positive or both negative → consistent."""
    assert _sign_consistent(10.0, 5.0) is True
    assert _sign_consistent(-10.0, -5.0) is True


def test_sign_consistent_opposite_signs() -> None:
    """Opposite signs → inconsistent."""
    assert _sign_consistent(-5.0, 5.0) is False
    assert _sign_consistent(5.0, -5.0) is False


def test_sign_consistent_zero() -> None:
    """Zero corrected or expected → inconsistent (product is 0, not > 0)."""
    assert _sign_consistent(0.0, 5.0) is False
    assert _sign_consistent(5.0, 0.0) is False


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
    noisy = [62.0, 120.0, -55.0, -110.0]  # not perfectly proportional
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
    bad_rows = _make_gain_rows(
        omegas=[0.5], k_actual=99.0, quality=1
    )  # should be ignored
    result = analyse_runs(good_rows + bad_rows, config)
    # Only 2 good gain rows (1 omega × CCW + CW), not the 2 bad ones
    assert result["n_samples"] == 2


def test_analyse_runs_excludes_sign_inconsistent_rows() -> None:
    """Gain rows where corrected sign differs from expected sign are excluded.

    A CCW command with negative corrected_delta_deg must be excluded even
    when quality_flag == 0.
    """
    config = _make_drive_cal_config()
    t0 = time.monotonic()

    # Good: CCW, positive corrected delta (signs match)
    good_ccw = _build_gain_row(
        omega=1.0,
        direction="ccw",
        duration_s=2.0,
        b_before=0.0,
        b_after=10.0,
        corrected_delta=10.0,
        expected_angle=math.degrees(1.0 * 2.0),
        qflag=0,
        t0=t0,
    )
    # Bad: CCW command but negative corrected delta (sign flip)
    bad_ccw = _build_gain_row(
        omega=1.0,
        direction="ccw",
        duration_s=2.0,
        b_before=0.0,
        b_after=-5.0,
        corrected_delta=-5.0,
        expected_angle=math.degrees(1.0 * 2.0),
        qflag=0,
        t0=t0,
    )
    # Good: CW, negative corrected delta (signs match)
    good_cw = _build_gain_row(
        omega=0.5,
        direction="cw",
        duration_s=2.0,
        b_before=0.0,
        b_after=-5.0,
        corrected_delta=-5.0,
        expected_angle=math.degrees(-0.5 * 2.0),
        qflag=0,
        t0=t0,
    )

    result = analyse_runs([good_ccw, bad_ccw, good_cw], config)
    assert result["n_samples"] == 2  # bad_ccw excluded
    assert "error" not in result


def test_analyse_runs_identity_gain() -> None:
    """Synthetic data with k=1 → turn_rate_gain ≈ 1.0."""
    config = _make_drive_cal_config()
    rows = _make_gain_rows(k_actual=1.0)
    result = analyse_runs(rows, config)
    assert result["turn_rate_gain"] == pytest.approx(1.0, rel=1e-4)
    assert result["effective_track_width_m"] == pytest.approx(
        config.ugv_track_width_nom, rel=1e-4
    )


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
    rows = _make_gain_rows(quality=1)  # all bad quality
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


def test_analyse_runs_dead_band_detected_with_string_moved_values() -> None:
    """Dead-band rows using string moved values ("0"/"1") are parsed correctly."""
    config = _make_drive_cal_config(noise_floor_deg=1.5)
    t0 = time.monotonic()
    rows = _make_gain_rows() + [
        _build_dead_band_row(
            omega=0.30,
            direction="ccw",
            duration_s=1.0,
            b_before=0.0,
            b_after=4.0,
            moved=True,
            qflag=0,
            t0=t0,
        ),
        _build_dead_band_row(
            omega=0.20,
            direction="ccw",
            duration_s=1.0,
            b_before=0.0,
            b_after=0.2,
            moved=False,
            qflag=0,
            t0=t0,
        ),
        _build_dead_band_row(
            omega=0.30,
            direction="cw",
            duration_s=1.0,
            b_before=0.0,
            b_after=-4.0,
            moved=True,
            qflag=0,
            t0=t0,
        ),
        _build_dead_band_row(
            omega=0.20,
            direction="cw",
            duration_s=1.0,
            b_before=0.0,
            b_after=-0.2,
            moved=False,
            qflag=0,
            t0=t0,
        ),
    ]
    result = analyse_runs(rows, config)
    assert result["angular_dead_band_rad_s"] == pytest.approx(0.25, rel=1e-4)


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
            omega=0.5,
            direction="ccw",
            duration_s=2.0,
            b_before=1.23,
            b_after=58.6,
            corrected_delta=55.4,
            expected_angle=57.3,
            qflag=0,
            t0=t0,
        ),
        _build_gain_row(
            omega=0.5,
            direction="cw",
            duration_s=2.0,
            b_before=58.6,
            b_after=1.3,
            corrected_delta=-55.2,
            expected_angle=-57.3,
            qflag=0,
            t0=t0,
        ),
        _build_dead_band_row(
            omega=0.2,
            direction="ccw",
            duration_s=1.5,
            b_before=0.0,
            b_after=3.5,
            moved=True,
            qflag=0,
            t0=t0,
        ),
        _build_dead_band_row(
            omega=0.1,
            direction="cw",
            duration_s=1.5,
            b_before=0.0,
            b_after=0.3,
            moved=False,
            qflag=0,
            t0=t0,
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
    assert gain_row["moved"] is None  # empty for gain_sweep

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
    assert loaded[0]["match_score_before"] == pytest.approx(-1.0, rel=1e-4)
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
            {},  # no ugv_drive key
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


@patch("tools.calibration.calibrate_ugv_drive._collect_click_bearing", return_value=5.0)
@patch("tools.calibration.calibrate_ugv_drive.time.sleep")
def test_run_sweep_row_count(
    mock_sleep: MagicMock, mock_click_bearing: MagicMock
) -> None:
    """2 omega commands + 2 dead-band steps → 4 gain rows + 4 dead-band rows = 8 total."""
    config = _make_drive_cal_config(
        omega_commands=(0.5, 1.0),
        dead_band_steps=(0.2, 0.1),
    )
    state = CalibrationStateContainer()
    ugv_mock = MagicMock()

    recenter_event = threading.Event()
    state.transition_to = _make_auto_unblock_transition(  # type: ignore[method-assign]
        state, recenter_event, []
    )
    _run_sweep(
        ugv_mock,
        config,
        state,
        threading.Event(),
        recenter_event,
        time.monotonic(),
    )

    rows = state.get_sweep_rows()
    assert rows is not None
    assert len(rows) == 8

    gain_rows = [r for r in rows if r["run_type"] == "gain_sweep"]
    dead_rows = [r for r in rows if r["run_type"] == "dead_band"]
    assert len(gain_rows) == 4  # 2 omegas × CCW + CW
    assert len(dead_rows) == 4  # 2 dead-band steps × CCW + CW


@patch("tools.calibration.calibrate_ugv_drive._collect_click_bearing", return_value=5.0)
@patch("tools.calibration.calibrate_ugv_drive.time.sleep")
def test_run_sweep_ugv_move_and_stop_calls(
    mock_sleep: MagicMock, mock_click_bearing: MagicMock
) -> None:
    """Each rotation run issues exactly one ugv.move and one ugv.stop."""
    config = _make_drive_cal_config(
        omega_commands=(0.5,),
        dead_band_steps=(0.2,),
    )
    state = CalibrationStateContainer()
    ugv_mock = MagicMock()

    recenter_event = threading.Event()
    state.transition_to = _make_auto_unblock_transition(  # type: ignore[method-assign]
        state, recenter_event, []
    )
    _run_sweep(
        ugv_mock,
        config,
        state,
        threading.Event(),
        recenter_event,
        time.monotonic(),
    )

    # 1 omega: 1 CCW + 1 CW = 2 gain runs
    # 1 dead_band step: 1 CCW + 1 CW = 2 dead-band runs
    # Total: 4 move calls, 4 stop calls
    assert ugv_mock.move.call_count == 4
    assert ugv_mock.stop.call_count == 4


@patch("tools.calibration.calibrate_ugv_drive._collect_click_bearing", return_value=5.0)
@patch("tools.calibration.calibrate_ugv_drive.time.sleep")
def test_run_sweep_ccw_and_cw_signs(
    mock_sleep: MagicMock, mock_click_bearing: MagicMock
) -> None:
    """CCW run uses +omega; CW run uses -omega."""
    config = _make_drive_cal_config(omega_commands=(0.7,), dead_band_steps=())
    state = CalibrationStateContainer()
    ugv_mock = MagicMock()

    recenter_event = threading.Event()
    state.transition_to = _make_auto_unblock_transition(  # type: ignore[method-assign]
        state, recenter_event, []
    )
    _run_sweep(
        ugv_mock,
        config,
        state,
        threading.Event(),
        recenter_event,
        time.monotonic(),
    )

    move_args = [call.args for call in ugv_mock.move.call_args_list]
    # CCW: move(0.0, +0.7); CW: move(0.0, -0.7)
    assert move_args[0] == (0.0, pytest.approx(0.7))
    assert move_args[1] == (0.0, pytest.approx(-0.7))


@patch("tools.calibration.calibrate_ugv_drive._collect_click_bearing", return_value=5.0)
@patch("tools.calibration.calibrate_ugv_drive.time.sleep")
def test_run_sweep_gain_block_ordering(
    mock_sleep: MagicMock, mock_click_bearing: MagicMock
) -> None:
    """All CCW gain runs complete before any CW gain run starts.

    With 2 omega commands the expected row order is:
    CCW@omega_0, CCW@omega_1, CW@omega_0, CW@omega_1.
    """
    config = _make_drive_cal_config(
        omega_commands=(0.5, 1.0),
        dead_band_steps=(),
    )
    state = CalibrationStateContainer()
    ugv_mock = MagicMock()

    recenter_event = threading.Event()
    state.transition_to = _make_auto_unblock_transition(  # type: ignore[method-assign]
        state, recenter_event, []
    )
    _run_sweep(
        ugv_mock,
        config,
        state,
        threading.Event(),
        recenter_event,
        time.monotonic(),
    )

    rows = state.get_sweep_rows()
    assert rows is not None
    gain_rows = [r for r in rows if r["run_type"] == "gain_sweep"]
    assert len(gain_rows) == 4
    directions = [r["direction"] for r in gain_rows]
    assert directions == ["ccw", "ccw", "cw", "cw"]


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


# ---------------------------------------------------------------------------
# Cancellation sentinel
# ---------------------------------------------------------------------------


class TestCheckCancel:
    def test_raises_when_set(self) -> None:
        event = threading.Event()
        event.set()
        with pytest.raises(_SweepCancelled):
            _check_cancel(event)

    def test_no_raise_when_clear(self) -> None:
        event = threading.Event()
        _check_cancel(event)  # should not raise


# ---------------------------------------------------------------------------
# Recenter pause — state transitions and event wiring
# ---------------------------------------------------------------------------


def _make_auto_unblock_transition(
    state: CalibrationStateContainer,
    recenter_event: threading.Event,
    states_seen: list[str],
) -> Any:
    """Return a transition_to replacement that auto-unblocks each recenter pause.

    Records every state transition in ``states_seen``.  When a
    ``WAITING_RECENTER`` transition fires it immediately calls
    ``try_start_recenter_resume()`` and sets ``recenter_event``, simulating
    an operator pressing ``GET /confirm``.
    """
    original_transition = state.transition_to

    def _capturing(new_status: SweepStatus) -> None:
        states_seen.append(new_status.state.value)
        original_transition(new_status)
        if new_status.state == CalibrationState.WAITING_RECENTER:
            state.try_start_recenter_resume()
            recenter_event.set()

    return _capturing


@patch("tools.calibration.calibrate_ugv_drive._collect_click_bearing", return_value=5.0)
@patch("tools.calibration.calibrate_ugv_drive.time.sleep")
def test_run_sweep_pauses_for_recenter(
    mock_sleep: MagicMock, mock_click_bearing: MagicMock
) -> None:
    """_run_sweep transitions to WAITING_RECENTER after every step except the last.

    With 1 gain omega and 1 dead-band omega there are 4 total steps, so there
    must be exactly 3 recenter pauses.  The auto-unblock helper simulates the
    operator pressing ``GET /confirm`` after each pause.
    """
    config = _make_drive_cal_config(
        omega_commands=(0.5,),
        dead_band_steps=(0.2,),
    )
    state = CalibrationStateContainer()

    recenter_event = threading.Event()
    states_seen: list[str] = []
    state.transition_to = _make_auto_unblock_transition(  # type: ignore[method-assign]
        state, recenter_event, states_seen
    )

    _run_sweep(
        MagicMock(),
        config,
        state,
        threading.Event(),
        recenter_event,
        time.monotonic(),
    )

    assert states_seen.count(CalibrationState.WAITING_RECENTER.value) == 3
    assert states_seen[-1] == CalibrationState.COMPLETE.value


@patch("tools.calibration.calibrate_ugv_drive._collect_click_bearing", return_value=5.0)
@patch("tools.calibration.calibrate_ugv_drive.time.sleep")
def test_recenter_pause_count_matches_block_count(
    mock_sleep: MagicMock, mock_click_bearing: MagicMock
) -> None:
    """Recenter pause positions follow the after-every-step rule.

    With 2 gain commands and 3 dead-band steps the 4 blocks have sizes
    2, 2, 3, 3 (total 10 steps).  Every completed step except the final
    one must pause for recenter, so expected pause steps are [1..9].
    """
    config = _make_drive_cal_config(
        omega_commands=(1.0, 2.0),
        dead_band_steps=(0.5, 1.0, 1.5),
    )
    state = CalibrationStateContainer()

    recenter_event = threading.Event()
    states_seen: list[str] = []
    pause_steps: list[int] = []

    original_transition = state.transition_to

    def capturing_transition(new_status: SweepStatus) -> None:
        states_seen.append(new_status.state.value)
        original_transition(new_status)
        if new_status.state == CalibrationState.WAITING_RECENTER:
            pause_steps.append(new_status.current_step)
            state.try_start_recenter_resume()
            recenter_event.set()

    state.transition_to = capturing_transition  # type: ignore[method-assign]

    _run_sweep(
        MagicMock(),
        config,
        state,
        threading.Event(),
        recenter_event,
        time.monotonic(),
    )

    assert pause_steps == [1, 2, 3, 4, 5, 6, 7, 8, 9]
    assert states_seen[-1] == CalibrationState.COMPLETE.value


@patch("tools.calibration.calibrate_ugv_drive._collect_click_bearing", return_value=5.0)
@patch("tools.calibration.calibrate_ugv_drive.time.sleep")
def test_recenter_event_cleared_between_pauses(
    mock_sleep: MagicMock, mock_click_bearing: MagicMock
) -> None:
    """A pre-set recenter_event does not allow any pause to be skipped.

    ``_wait_for_recenter`` clears the event before entering
    ``WAITING_RECENTER``, so even if the event is already set at the start
    of each pause all 3 pauses must still be entered.
    """
    config = _make_drive_cal_config(
        omega_commands=(0.5,),
        dead_band_steps=(0.2,),
    )
    state = CalibrationStateContainer()

    recenter_event = threading.Event()
    recenter_event.set()  # pre-set: would skip pauses if clear() were absent

    states_seen: list[str] = []
    state.transition_to = _make_auto_unblock_transition(  # type: ignore[method-assign]
        state, recenter_event, states_seen
    )

    _run_sweep(
        MagicMock(),
        config,
        state,
        threading.Event(),
        recenter_event,
        time.monotonic(),
    )

    assert states_seen.count(CalibrationState.WAITING_RECENTER.value) == 3
    assert states_seen[-1] == CalibrationState.COMPLETE.value


def test_try_start_recenter_resume_transitions_state() -> None:
    """try_start_recenter_resume returns True and preserves progress."""
    state = CalibrationStateContainer()
    state.transition_to(
        SweepStatus(
            state=CalibrationState.WAITING_RECENTER,
            progress_pct=52.5,
            current_step=3,
            total_steps=6,
        )
    )

    result = state.try_start_recenter_resume()

    assert result is True
    snap = state.get_snapshot()
    assert snap.state == CalibrationState.SWEEPING
    assert snap.progress_pct == pytest.approx(52.5)
    assert snap.current_step == 3
    assert snap.total_steps == 6


def test_try_start_recenter_resume_fails_in_wrong_state() -> None:
    """try_start_recenter_resume returns False when not in WAITING_RECENTER."""
    state = CalibrationStateContainer()  # starts in WAITING_CLICK
    assert state.try_start_recenter_resume() is False


def test_try_reset_blocked_in_waiting_recenter() -> None:
    """try_reset returns False when state is WAITING_RECENTER."""
    state = CalibrationStateContainer()
    state.transition_to(SweepStatus(state=CalibrationState.WAITING_RECENTER))
    assert state.try_reset() is False


def test_cancel_sweep_sets_recenter_event() -> None:
    """cancel_sweep sets both cancel and recenter events."""
    state = CalibrationStateContainer()
    orchestrator = CalibrationOrchestrator(
        config=_make_drive_cal_config(),
        ugv=MagicMock(),
        state=state,
    )

    orchestrator.cancel_sweep()

    assert orchestrator._cancel_event.is_set()
    assert orchestrator._recenter_event.is_set()


@patch("tools.calibration.calibrate_ugv_drive._collect_click_bearing", return_value=5.0)
@patch("tools.calibration.calibrate_ugv_drive.time.sleep")
def test_run_sweep_stall_gate_sets_flag(
    mock_sleep: MagicMock, mock_click_bearing: MagicMock
) -> None:
    """Gain steps with zero bearing change have the stall flag (bit 2) set.

    When _collect_click_bearing always returns 5.0 (same bearing before and
    after rotation), delta = 0 — below any noise_floor_deg — so the stall
    gate must set quality_flag bit 2 on every gain row.
    """
    config = _make_drive_cal_config(
        omega_commands=(1.0,),
        dead_band_steps=(),
        noise_floor_deg=2.0,
    )
    state = CalibrationStateContainer()

    recenter_event = threading.Event()
    state.transition_to = _make_auto_unblock_transition(  # type: ignore[method-assign]
        state, recenter_event, []
    )

    _run_sweep(
        MagicMock(),
        config,
        state,
        threading.Event(),
        recenter_event,
        time.monotonic(),
    )

    rows = state.get_sweep_rows()
    assert rows is not None
    gain_rows = [r for r in rows if r["run_type"] == "gain_sweep"]
    assert len(gain_rows) == 2  # 1 omega × CCW + CW
    assert all(r["quality_flag"] & 4 != 0 for r in gain_rows)


def test_get_status_text_waiting_recenter() -> None:
    """_get_status_text returns yellow recenter guidance for WAITING_RECENTER."""
    status = {
        "state": CalibrationState.WAITING_RECENTER.value,
        "current_step": 3,
        "total_steps": 10,
        "progress_pct": 30.0,
        "error_message": None,
    }
    result = _get_status_text(CalibrationState.WAITING_RECENTER.value, status)
    assert result is not None
    text, colour, _scale = result
    assert "Re-centre" in text
    assert "/confirm" in text
    assert "3/10" in text
    assert colour == (0, 255, 255)  # yellow in BGR


# ---------------------------------------------------------------------------
# _bearing_from_distorted_click
# ---------------------------------------------------------------------------


def test_bearing_from_distorted_click_centre_is_zero() -> None:
    """Principal point click → bearing of exactly 0°."""
    K, D = _make_K_D_zero_distortion()
    assert _bearing_from_distorted_click(320.0, 240.0, K, D) == pytest.approx(
        0.0, abs=1e-6
    )


def test_bearing_from_distorted_click_right_of_centre_positive() -> None:
    """Click to the right of optical axis → positive bearing."""
    K, D = _make_K_D_zero_distortion()
    assert _bearing_from_distorted_click(420.0, 240.0, K, D) > 0.0


def test_bearing_from_distorted_click_left_of_centre_negative() -> None:
    """Click to the left of optical axis → negative bearing."""
    K, D = _make_K_D_zero_distortion()
    assert _bearing_from_distorted_click(220.0, 240.0, K, D) < 0.0


# ---------------------------------------------------------------------------
# _get_status_text — WAITING_CLICK
# ---------------------------------------------------------------------------


def test_get_status_text_waiting_click() -> None:
    """_get_status_text returns the click_prompt text for WAITING_CLICK."""
    status = {
        "state": CalibrationState.WAITING_CLICK.value,
        "click_prompt": "Click the marker to begin calibration",
        "current_step": 0,
        "total_steps": 8,
        "progress_pct": 0.0,
        "error_message": None,
    }
    result = _get_status_text(CalibrationState.WAITING_CLICK.value, status)
    assert result is not None
    text, colour, _scale = result
    assert "Click" in text
    assert colour == (0, 255, 0)


# ---------------------------------------------------------------------------
# try_reset blocked in WAITING_CLICK
# ---------------------------------------------------------------------------


def test_try_reset_blocked_in_waiting_click() -> None:
    """try_reset returns False when state is WAITING_CLICK (sweep thread active)."""
    state = CalibrationStateContainer()
    # Default initial state is WAITING_CLICK
    assert state.get_snapshot().state == CalibrationState.WAITING_CLICK
    assert state.try_reset() is False


# ---------------------------------------------------------------------------
# _try_reopen_capture
# ---------------------------------------------------------------------------


@patch("tools.calibration.calibrate_ugv_drive.time.sleep")
@patch("tools.calibration.calibrate_ugv_drive.ensure_camera_device_available")
def test_try_reopen_capture_success(
    mock_preflight: MagicMock, mock_sleep: MagicMock
) -> None:
    """Reopen performs preflight, opens device, and reapplies capture settings."""
    cap = MagicMock()
    cap.isOpened.return_value = True

    _try_reopen_capture(cap, "/dev/video0", 1920, 1080)

    cap.release.assert_called_once()
    mock_preflight.assert_called_once_with("/dev/video0")
    cap.open.assert_called_once_with("/dev/video0", cv2.CAP_V4L2)
    cap.set.assert_any_call(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set.assert_any_call(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set.assert_any_call(cv2.CAP_PROP_AUTO_EXPOSURE, 3)


@patch("tools.calibration.calibrate_ugv_drive.time.sleep")
@patch("tools.calibration.calibrate_ugv_drive.ensure_camera_device_available")
def test_try_reopen_capture_preflight_failure_raises(
    mock_preflight: MagicMock, mock_sleep: MagicMock
) -> None:
    """Preflight failure is surfaced as a reopen RuntimeError with chaining."""
    cap = MagicMock()
    mock_preflight.side_effect = RuntimeError("busy")

    with pytest.raises(RuntimeError, match="Failed to reopen camera device"):
        _try_reopen_capture(cap, "/dev/video0", 1920, 1080)

    cap.open.assert_not_called()
