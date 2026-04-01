"""Automated pytest tests for non-hardware logic in calibrate_pantilt_servo.

All tests run without any physical hardware.  They import pure functions and
CSV/YAML helpers directly from the calibration script.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import TypeAlias

import pytest
import yaml

from tools.calibration.calibrate_pantilt_servo import (
    _load_csv,
    _validate_geometry,
    _write_csv,
    _write_results_atomic,
    analyse_sweep,
    centroid_to_angle,
    compute_hysteresis,
    correct_for_forward_offset,
    estimate_dead_band,
    fit_linear,
    fit_piecewise_linear,
    quality_flag,
)


RowValue: TypeAlias = float | int | str


# ---------------------------------------------------------------------------
# centroid_to_angle
# ---------------------------------------------------------------------------


def test_centroid_to_angle_zero() -> None:
    """u == cx → phi must be exactly 0.0 regardless of fx."""
    assert centroid_to_angle(u_px=500.0, cx=500.0, fx=544.64) == pytest.approx(0.0)


def test_centroid_to_angle_right() -> None:
    """u > cx (target shifted right of optical axis) → phi < 0 (default sign convention).

    When the camera pans right, the target moves LEFT in the image (u < cx).
    Conversely, if the target is to the RIGHT of cx (u > cx), the camera has
    effectively panned left, so phi should be negative.
    """
    phi = centroid_to_angle(u_px=600.0, cx=500.0, fx=544.64)
    assert phi < 0.0


def test_centroid_to_angle_left() -> None:
    """u < cx (target shifted left of optical axis) → phi > 0."""
    phi = centroid_to_angle(u_px=400.0, cx=500.0, fx=544.64)
    assert phi > 0.0


def test_centroid_to_angle_known_value() -> None:
    """Verify against manually computed arctan result."""
    # atan2(100, 544.64) ≈ 0.1822 rad ≈ 10.44°; phi = -10.44°
    expected = -math.degrees(math.atan2(100.0, 544.64))
    assert centroid_to_angle(u_px=600.0, cx=500.0, fx=544.64) == pytest.approx(expected)


# ---------------------------------------------------------------------------
# correct_for_forward_offset
# ---------------------------------------------------------------------------


def test_correct_for_forward_offset_zero_offset() -> None:
    """d=0 → corrected angle equals naive angle (identity)."""
    assert correct_for_forward_offset(10.0, d_m=0.0, D_m=2.0) == pytest.approx(10.0)


def test_correct_for_forward_offset_known_value() -> None:
    """Spot-check against manually computed value."""
    phi_cam = 10.0
    d, D = 0.05, 2.0
    expected = phi_cam - math.degrees(
        math.asin((d / D) * math.sin(math.radians(phi_cam)))
    )
    assert correct_for_forward_offset(phi_cam, d, D) == pytest.approx(expected)


def test_correct_for_forward_offset_negative_phi() -> None:
    """Correction is anti-symmetric: negative phi corrects symmetrically."""
    pos = correct_for_forward_offset(10.0, 0.05, 2.0)
    neg = correct_for_forward_offset(-10.0, 0.05, 2.0)
    assert neg == pytest.approx(-pos)


def test_correct_for_forward_offset_compression() -> None:
    """Corrected angle is strictly smaller in magnitude than naive when d > 0."""
    phi_corrected = correct_for_forward_offset(20.0, d_m=0.05, D_m=2.0)
    assert abs(phi_corrected) < 20.0


def test_correct_for_forward_offset_domain_error() -> None:
    """Arcsin domain violation (|d/D·sin(phi)| >= 1) → ValueError."""
    with pytest.raises(ValueError, match="Trig domain violation"):
        # sin(90°)=1, d/D=1 → ratio exactly 1.0
        correct_for_forward_offset(90.0, d_m=2.0, D_m=2.0)


# ---------------------------------------------------------------------------
# _validate_geometry
# ---------------------------------------------------------------------------


def test_validate_geometry_valid() -> None:
    """Valid inputs must not raise."""
    _validate_geometry(0.04, 2.0)


def test_validate_geometry_non_positive_distance() -> None:
    """calibration_target_distance_m <= 0 → ValueError."""
    with pytest.raises(
        ValueError, match="calibration_target_distance_m must be positive"
    ):
        _validate_geometry(0.04, 0.0)
    with pytest.raises(
        ValueError, match="calibration_target_distance_m must be positive"
    ):
        _validate_geometry(0.04, -1.0)


def test_validate_geometry_negative_offset() -> None:
    """camera_forward_offset_m < 0 → ValueError."""
    with pytest.raises(
        ValueError, match="camera_forward_offset_m must be non-negative"
    ):
        _validate_geometry(-0.01, 2.0)


def test_validate_geometry_offset_exceeds_distance() -> None:
    """camera_forward_offset_m >= calibration_target_distance_m → ValueError."""
    with pytest.raises(ValueError):
        _validate_geometry(3.0, 2.0)
    with pytest.raises(ValueError):
        _validate_geometry(2.0, 2.0)


# ---------------------------------------------------------------------------
# quality_flag
# ---------------------------------------------------------------------------


def test_quality_flag_good() -> None:
    """High match score, centroid well within frame → flag 0."""
    assert quality_flag(match_score=0.9, u_px=640.0, cam_width=1280) == 0


def test_quality_flag_low_match() -> None:
    """Match score below threshold → flag 1."""
    assert quality_flag(match_score=0.3, u_px=640.0, cam_width=1280) == 1


def test_quality_flag_near_edge() -> None:
    """Centroid within edge_margin_px of frame boundary → flag 2."""
    assert quality_flag(match_score=0.9, u_px=5.0, cam_width=1280) == 2
    assert quality_flag(match_score=0.9, u_px=1275.0, cam_width=1280) == 2


def test_quality_flag_both() -> None:
    """Low score AND near edge → flag 3."""
    assert quality_flag(match_score=0.1, u_px=5.0, cam_width=1280) == 3


# ---------------------------------------------------------------------------
# estimate_dead_band
# ---------------------------------------------------------------------------


def test_dead_band_symmetric() -> None:
    """Commands ±1 produce no motion; ±3 and beyond produce motion."""
    cmds = [-5.0, -3.0, -1.0, 0.0, 1.0, 3.0, 5.0]
    phis = [-4.2, -2.5, 0.0, 0.0, 0.0, 2.4, 4.1]
    pos_db, neg_db = estimate_dead_band(cmds, phis, noise_floor_deg=0.5)
    # Smallest positive cmd with |phi| > 0.5 is 3.0 (phi=2.4)
    assert pos_db == pytest.approx(3.0)
    # Smallest magnitude negative cmd with |phi| > 0.5 is -3.0 (phi=-2.5)
    assert neg_db == pytest.approx(-3.0)


def test_dead_band_no_motion() -> None:
    """All phi values below noise floor → both sides return None."""
    cmds = [-10.0, -5.0, 0.0, 5.0, 10.0]
    phis = [0.1, 0.0, 0.0, 0.0, 0.1]
    pos_db, neg_db = estimate_dead_band(cmds, phis, noise_floor_deg=0.5)
    assert pos_db is None
    assert neg_db is None


def test_dead_band_one_sided() -> None:
    """Only the positive side produces motion."""
    cmds = [-5.0, 0.0, 5.0, 10.0]
    phis = [0.0, 0.0, 4.5, 8.0]
    pos_db, neg_db = estimate_dead_band(cmds, phis, noise_floor_deg=0.5)
    assert pos_db == pytest.approx(5.0)
    assert neg_db is None


# ---------------------------------------------------------------------------
# fit_linear
# ---------------------------------------------------------------------------


def test_fit_linear_identity() -> None:
    """phi = cmd exactly → slope=1, intercept=0, MAE=0."""
    cmds = [-10.0, -5.0, 0.0, 5.0, 10.0]
    phis = [-10.0, -5.0, 0.0, 5.0, 10.0]
    result = fit_linear(cmds, phis)
    assert result["slope"] == pytest.approx(1.0, abs=1e-6)
    assert result["intercept"] == pytest.approx(0.0, abs=1e-6)
    assert result["mae_deg"] == pytest.approx(0.0, abs=1e-6)
    assert result["max_abs_error_deg"] == pytest.approx(0.0, abs=1e-6)


def test_fit_linear_constant_offset() -> None:
    """phi = cmd + 1 → intercept=1, slope=1, MAE=0."""
    cmds = [-10.0, -5.0, 0.0, 5.0, 10.0]
    phis = [-9.0, -4.0, 1.0, 6.0, 11.0]
    result = fit_linear(cmds, phis)
    assert result["slope"] == pytest.approx(1.0, abs=1e-6)
    assert result["intercept"] == pytest.approx(1.0, abs=1e-6)
    assert result["mae_deg"] == pytest.approx(0.0, abs=1e-6)


# ---------------------------------------------------------------------------
# fit_piecewise_linear
# ---------------------------------------------------------------------------


def test_fit_piecewise_linear_identity() -> None:
    """Samples on identity line → command_samples == angle_samples."""
    cmds = [-10.0, -5.0, 0.0, 5.0, 10.0]
    phis = [-10.0, -5.0, 0.0, 5.0, 10.0]
    result = fit_piecewise_linear(cmds, phis)
    assert result["command_samples"] == cmds
    assert result["angle_samples"] == phis


def test_fit_piecewise_linear_sorted() -> None:
    """Unsorted input should produce sorted command_samples."""
    cmds = [5.0, -5.0, 0.0]
    phis = [4.5, -4.5, 0.0]
    result = fit_piecewise_linear(cmds, phis)
    assert result["command_samples"] == [-5.0, 0.0, 5.0]


def test_fit_piecewise_linear_outlier_max_error() -> None:
    """Outlier at centre causes max_abs_error to exceed mae."""
    cmds = [-10.0, -5.0, 0.0, 5.0, 10.0]
    phis = [-10.0, -5.0, 3.0, 5.0, 10.0]  # centre point 3 deg above the regression line
    result = fit_piecewise_linear(cmds, phis)
    # Leave-one-out: residuals include the centre outlier (3 deg) and boundary
    # clamping artefacts from np.interp when endpoints are excluded.  The key
    # property is that max_abs_error is strictly larger than mae.
    assert result["max_abs_error_deg"] > result["mae_deg"]
    assert result["max_abs_error_deg"] > 0.0


def test_fit_piecewise_linear_few_samples_guard() -> None:
    """Fewer than 3 samples returns 0.0 metrics without raising."""
    result = fit_piecewise_linear([0.0, 5.0], [0.0, 4.5])
    assert result["mae_deg"] == pytest.approx(0.0)
    assert result["max_abs_error_deg"] == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# compute_hysteresis
# ---------------------------------------------------------------------------


def test_compute_hysteresis_zero() -> None:
    """Identical forward and reverse sweeps → hysteresis = 0."""
    cmds = [-5.0, 0.0, 5.0]
    assert compute_hysteresis(cmds, cmds, cmds, cmds) == pytest.approx(0.0)


def test_compute_hysteresis_known() -> None:
    """Known offset between forward and reverse → correct mean."""
    fwd_cmds = [-5.0, 0.0, 5.0]
    fwd_phis = [-4.0, 0.0, 4.0]
    rev_cmds = [-5.0, 0.0, 5.0]
    rev_phis = [-3.0, 0.5, 5.0]  # differences: 1.0, 0.5, 1.0
    result = compute_hysteresis(fwd_cmds, fwd_phis, rev_cmds, rev_phis)
    assert result == pytest.approx((1.0 + 0.5 + 1.0) / 3.0, abs=1e-6)


def test_compute_hysteresis_no_match() -> None:
    """No overlapping command values → hysteresis = 0.0."""
    result = compute_hysteresis([1.0, 2.0], [1.0, 2.0], [3.0, 4.0], [3.0, 4.0])
    assert result == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# CSV round-trip
# ---------------------------------------------------------------------------


def _make_sample_row(**overrides: RowValue) -> dict[str, RowValue]:
    row: dict[str, RowValue] = {
        "timestamp_s": 1.0,
        "commanded_pan_deg": -5.0,
        "tilt_setpoint_deg": 0.0,
        "sweep_direction": "forward",
        "u_px": 450.0,
        "fx_px": 544.64,
        "cx_px": 500.0,
        "image_angle_deg": -5.25,
        "phi_deg": 5.25,
        "settle_time_s": 1.5,
        "frames_averaged": 10,
        "match_score": 0.85,
        "quality_flag": 0,
    }
    row.update(overrides)
    return row


def test_csv_roundtrip(tmp_path: Path) -> None:
    """Write a minimal CSV, reload it, verify numeric types and values."""
    rows = [_make_sample_row()]
    csv_path = tmp_path / "test.csv"
    _write_csv(csv_path, rows)
    loaded = _load_csv(csv_path)

    assert len(loaded) == 1
    r = loaded[0]
    assert r["commanded_pan_deg"] == pytest.approx(-5.0)
    assert r["phi_deg"] == pytest.approx(5.25)
    assert r["quality_flag"] == 0  # must be int, not str
    assert r["frames_averaged"] == 10  # must be int
    assert r["sweep_direction"] == "forward"  # must remain str


def test_csv_roundtrip_multiple_rows(tmp_path: Path) -> None:
    """Multiple rows with both directions survive a write/read cycle."""
    rows = [
        _make_sample_row(
            commanded_pan_deg=-5.0, sweep_direction="forward", quality_flag=0
        ),
        _make_sample_row(
            commanded_pan_deg=5.0, sweep_direction="reverse", quality_flag=1
        ),
    ]
    csv_path = tmp_path / "multi.csv"
    _write_csv(csv_path, rows)
    loaded = _load_csv(csv_path)

    assert len(loaded) == 2
    assert loaded[0]["sweep_direction"] == "forward"
    assert loaded[1]["sweep_direction"] == "reverse"
    assert loaded[1]["quality_flag"] == 1


# ---------------------------------------------------------------------------
# Atomic YAML write
# ---------------------------------------------------------------------------


def test_atomic_write_preserves_other_keys(tmp_path: Path) -> None:
    """Writing pan_tilt_servo must not disturb other top-level keys."""
    sensor_config = tmp_path / "sensor_config.yaml"
    original = {
        "camera": {"fps": 30},
        "lidar": {"port": "/dev/ttyUSB0"},
        "ugv": {"port": "/dev/ttyAMA0"},
        "extrinsic": {"lidar_to_pantilt_offset_deg": 2.8},
    }
    sensor_config.write_text(yaml.dump(original))

    pan_tilt_servo_dict = {
        "calibrated_at": "2026-03-31T12:00:00",
        "calibration_method": "servo_curve_sweep",
        "n_good_samples": 10,
    }
    _write_results_atomic(sensor_config, pan_tilt_servo_dict)

    with sensor_config.open() as f:
        result = yaml.safe_load(f)

    assert result["camera"]["fps"] == 30
    assert result["lidar"]["port"] == "/dev/ttyUSB0"
    assert result["extrinsic"]["lidar_to_pantilt_offset_deg"] == pytest.approx(2.8)
    assert result["pan_tilt_servo"]["n_good_samples"] == 10
    assert result["pan_tilt_servo"]["calibration_method"] == "servo_curve_sweep"


def test_atomic_write_replaces_existing_pan_tilt_section(tmp_path: Path) -> None:
    """A second atomic write replaces the pan_tilt_servo key wholesale."""
    sensor_config = tmp_path / "sensor_config.yaml"
    sensor_config.write_text(
        yaml.dump({"camera": {"fps": 30}, "pan_tilt_servo": {"old_key": 99}})
    )
    _write_results_atomic(sensor_config, {"new_key": 42})

    with sensor_config.open() as f:
        result = yaml.safe_load(f)

    assert "old_key" not in result["pan_tilt_servo"]
    assert result["pan_tilt_servo"]["new_key"] == 42
    assert result["camera"]["fps"] == 30


# ---------------------------------------------------------------------------
# analyse_sweep integration
# ---------------------------------------------------------------------------


def _make_sweep_rows(n: int = 10) -> list[dict[str, RowValue]]:
    """Generate a minimal synthetic sweep dataset for integration tests."""
    rows = []
    # Forward: cmd -20 to +20 in steps of ~4.4
    import numpy as np

    cmds = list(np.linspace(-20.0, 20.0, n))
    for cmd in cmds:
        # Simulate phi ≈ 0.9 * cmd (positive slope, slight compression)
        phi = 0.9 * cmd
        rows.append(
            _make_sample_row(
                commanded_pan_deg=round(cmd, 4),
                phi_deg=round(phi, 4),
                sweep_direction="forward",
                quality_flag=0,
            )
        )
    # Reverse: same commands, slight hysteresis
    for cmd in reversed(cmds):
        phi = 0.9 * cmd + 0.5  # 0.5-deg hysteresis offset
        rows.append(
            _make_sample_row(
                commanded_pan_deg=round(cmd, 4),
                phi_deg=round(phi, 4),
                sweep_direction="reverse",
                quality_flag=0,
            )
        )
    return rows


def test_analyse_sweep_returns_required_keys() -> None:
    """analyse_sweep output must contain all keys expected by sensor_config.yaml."""
    rows = _make_sweep_rows(10)
    result = analyse_sweep(
        rows,
        noise_floor_deg=0.5,
        camera_forward_offset_m=0.04,
        calibration_target_distance_m=2.0,
    )

    required = {
        "calibration_method",
        "angle_measurement_model",
        "camera_forward_offset_m",
        "calibration_target_distance_m",
        "tilt_setpoint_deg",
        "dead_band_pos_deg",
        "dead_band_neg_deg",
        "phi_min_deg",
        "phi_max_deg",
        "hysteresis_mean_deg",
        "linear_fit",
        "piecewise_linear",
        "n_good_samples",
        "n_total_samples",
    }
    assert required.issubset(result.keys())


def test_analyse_sweep_no_good_samples() -> None:
    """All samples with quality_flag != 0 → n_good_samples == 0."""
    rows = _make_sweep_rows(5)
    for r in rows:
        r["quality_flag"] = 2
    result = analyse_sweep(
        rows,
        noise_floor_deg=0.5,
        camera_forward_offset_m=0.04,
        calibration_target_distance_m=2.0,
    )
    assert result["n_good_samples"] == 0
    assert "error" in result


def test_analyse_sweep_hysteresis_approx() -> None:
    """Synthetic 0.5-deg offset → hysteresis_mean_deg ≈ 0.5."""
    rows = _make_sweep_rows(10)
    result = analyse_sweep(
        rows,
        noise_floor_deg=0.1,
        camera_forward_offset_m=0.04,
        calibration_target_distance_m=2.0,
    )
    assert result["hysteresis_mean_deg"] == pytest.approx(0.5, abs=0.05)
