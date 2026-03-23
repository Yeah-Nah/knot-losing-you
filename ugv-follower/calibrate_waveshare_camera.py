"""Intrinsic calibration for the Waveshare RGB camera.

Loads a folder of checkerboard images captured by capture_calibration_images.py,
runs cv2.calibrateCamera, validates the results, and writes them into
configs/sensor_config.yaml under the ``waveshare_rgb`` key.

Can be run on the Pi or a laptop — no hardware connection required.

Usage
-----
::

    python calibrate_waveshare_camera.py \\
        --images calibration/images \\
        --square-mm 28

Arguments
---------
--images     Path to folder containing .jpg / .png calibration frames.
--square-mm  Physical side length of one checkerboard square in millimetres.
--config     (Optional) Path to calibration_config.yaml. Defaults to
             configs/calibration_config.yaml relative to this script.
--sensor-config
             (Optional) Path to sensor_config.yaml to write results into.
             Defaults to configs/sensor_config.yaml relative to this script.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from loguru import logger

_SCRIPT_DIR = Path(__file__).resolve().parent
_DEFAULT_CAL_CONFIG = _SCRIPT_DIR / "configs" / "calibration_config.yaml"
_DEFAULT_SENSOR_CONFIG = _SCRIPT_DIR / "configs" / "sensor_config.yaml"

_CORNER_CRITERIA = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001,
)

# Acceptance thresholds
_RMS_MAX = 1  # pixels
_PP_MARGIN = 0.10  # principal point must be within 10% of frame edge from centre
_FOCAL_RATIO_MAX = 0.05  # |fx - fy| / max(fx, fy) < 5%


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _load_board_size(cal_config_path: Path) -> tuple[int, int]:
    """Return (cols, rows) of inner corners from calibration_config.yaml."""
    with cal_config_path.open() as f:
        cfg = yaml.safe_load(f)
    cols, rows = cfg["checkerboard"]["inner_corners"]
    return int(cols), int(rows)


def _collect_image_paths(folder: Path) -> list[Path]:
    """Return a sorted list of .jpg and .png image paths found in *folder*.

    Parameters
    ----------
    folder : Path
        Directory to search (non-recursive).

    Returns
    -------
    list[Path]
        All ``*.jpg`` files (sorted) followed by all ``*.png`` files (sorted).
    """
    paths = sorted(folder.glob("*.jpg")) + sorted(folder.glob("*.png"))
    return paths


def _detect_corners(
    image_paths: list[Path],
    board_size: tuple[int, int],
) -> tuple[list[np.ndarray], list[np.ndarray], tuple[int, int]]:
    """Run corner detection on all images.

    Returns
    -------
    object_points : list of (N, 1, 3) float64 arrays (Z=0 board coordinates)
    image_points  : list of (N, 1, 2) float64 arrays (pixel coordinates)
    image_size    : (width, height) of the images
    """
    cols, rows = board_size
    # Board coordinates: (col, row, 0) for each inner corner.
    # Shape (N, 1, 3) float64 required by cv2.fisheye.calibrate.
    objp = np.zeros((cols * rows, 1, 3), dtype=np.float64)
    objp[:, 0, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    # Scale is applied in calibrateCamera via the square_mm argument elsewhere;
    # here we keep units as "squares" and pass square_mm as the object point scale.

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None
    skipped = 0

    for path in image_paths:
        img = cv2.imread(str(path))
        if img is None:
            logger.warning(f"Could not read {path} — skipping.")
            skipped += 1
            continue
        h, w = img.shape[:2]
        if image_size is None:
            image_size = (w, h)
        elif image_size != (w, h):
            logger.warning(
                f"{path.name}: size {w}×{h} differs from expected "
                f"{image_size[0]}×{image_size[1]} — skipping."
            )
            skipped += 1
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray,
            board_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if not found:
            logger.warning(f"{path.name}: checkerboard not detected — skipping.")
            skipped += 1
            continue

        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), _CORNER_CRITERIA)
        object_points.append(objp)
        image_points.append(corners.astype(np.float64))

    logger.info(
        f"Corner detection: {len(object_points)} accepted, {skipped} skipped "
        f"out of {len(image_paths)} images."
    )
    if image_size is None:
        raise RuntimeError("No valid images found — cannot determine image size.")
    return object_points, image_points, image_size


def _validate(
    rms: float,
    camera_matrix: np.ndarray,
    image_size: tuple[int, int],
) -> None:
    """Raise ``ValueError`` if calibration results fail acceptance criteria.

    Checks RMS re-projection error, principal point position (must sit within
    the central 80% of the frame), and focal length symmetry.

    Parameters
    ----------
    rms : float
        RMS re-projection error returned by the calibration call.
    camera_matrix : np.ndarray
        3×3 intrinsic matrix K.
    image_size : tuple[int, int]
        ``(width, height)`` of the calibration images in pixels.

    Raises
    ------
    ValueError
        If any acceptance criterion is not met; the message lists all failures.
    """
    errors: list[str] = []

    if rms >= _RMS_MAX:
        errors.append(f"RMS {rms:.4f} px ≥ threshold {_RMS_MAX} px")

    w, h = image_size
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    cx_min, cx_max = w * _PP_MARGIN, w * (1 - _PP_MARGIN)
    cy_min, cy_max = h * _PP_MARGIN, h * (1 - _PP_MARGIN)
    if not (cx_min <= cx <= cx_max):
        errors.append(
            f"Principal point cx={cx:.1f} outside [{cx_min:.0f}, {cx_max:.0f}] "
            f"(10% margins of {w} px width)"
        )
    if not (cy_min <= cy <= cy_max):
        errors.append(
            f"Principal point cy={cy:.1f} outside [{cy_min:.0f}, {cy_max:.0f}] "
            f"(10% margins of {h} px height)"
        )

    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    focal_ratio = abs(fx - fy) / max(fx, fy)
    if focal_ratio >= _FOCAL_RATIO_MAX:
        errors.append(
            f"Focal length asymmetry |fx-fy|/max = {focal_ratio:.3f} ≥ {_FOCAL_RATIO_MAX} "
            f"(fx={fx:.2f}, fy={fy:.2f})"
        )

    if errors:
        raise ValueError("Calibration failed validation:\n  " + "\n  ".join(errors))


def _write_results(
    sensor_config_path: Path,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    image_size: tuple[int, int],
    rms: float,
) -> None:
    """Merge calibration results into sensor_config.yaml under the ``waveshare_rgb`` key.

    Reads the existing file, replaces only the ``waveshare_rgb`` section, and
    writes the result back — all other top-level keys are preserved.

    Parameters
    ----------
    sensor_config_path : Path
        Path to ``sensor_config.yaml``.
    camera_matrix : np.ndarray
        3×3 intrinsic matrix K.
    dist_coeffs : np.ndarray
        Fisheye distortion coefficients shaped ``(4, 1)``.
    image_size : tuple[int, int]
        ``(width, height)`` of the calibration images in pixels.
    rms : float
        RMS re-projection error in pixels.
    """
    with sensor_config_path.open() as f:
        config = yaml.safe_load(f) or {}

    # Convert numpy arrays to plain Python lists for YAML serialisation
    k = camera_matrix.tolist()
    d = dist_coeffs.flatten().tolist()

    config["waveshare_rgb"] = {
        "model": "fisheye",
        "camera_matrix": k,
        "dist_coeffs": d,
        "resolution": list(image_size),
        "rms_reprojection_error": round(float(rms), 6),
    }

    with sensor_config_path.open("w") as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    logger.info(f"Results written to {sensor_config_path}")


# ---------------------------------------------------------------------------
# Entry point helpers
# ---------------------------------------------------------------------------


def _build_arg_parser() -> argparse.ArgumentParser:
    """Build and return the CLI argument parser for the calibration script.

    Returns
    -------
    argparse.ArgumentParser
        Configured parser with ``--images``, ``--square-mm``, ``--config``,
        and ``--sensor-config`` arguments.
    """
    parser = argparse.ArgumentParser(
        description="Run OpenCV checkerboard intrinsic calibration on saved images."
    )
    parser.add_argument(
        "--images",
        required=True,
        type=Path,
        help="Folder containing .jpg / .png calibration images.",
    )
    parser.add_argument(
        "--square-mm",
        required=True,
        type=float,
        help="Physical side length of one checkerboard square in millimetres.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=_DEFAULT_CAL_CONFIG,
        help=f"Path to calibration_config.yaml (default: {_DEFAULT_CAL_CONFIG}).",
    )
    parser.add_argument(
        "--sensor-config",
        type=Path,
        default=_DEFAULT_SENSOR_CONFIG,
        help=f"Path to sensor_config.yaml to write results into (default: {_DEFAULT_SENSOR_CONFIG}).",
    )
    return parser


def _run_fisheye_calibration(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    image_size: tuple[int, int],
) -> tuple[float, np.ndarray, np.ndarray]:
    """Run ``cv2.fisheye.calibrate`` and return the results.

    Parameters
    ----------
    object_points : list of np.ndarray
        Per-image board coordinates, each shaped ``(N, 1, 3)`` float64.
    image_points : list of np.ndarray
        Per-image detected corner coordinates, each shaped ``(N, 1, 2)`` float64.
    image_size : tuple[int, int]
        ``(width, height)`` of the calibration images in pixels.

    Returns
    -------
    rms : float
        RMS re-projection error in pixels.
    camera_matrix : np.ndarray
        3×3 intrinsic matrix K.
    dist_coeffs : np.ndarray
        Fisheye distortion coefficients ``[k1, k2, k3, k4]``, shaped ``(4, 1)``.
    """
    K = np.zeros((3, 3), dtype=np.float64)
    D = np.zeros((4, 1), dtype=np.float64)
    rms, K, D, _rvecs, _tvecs = cv2.fisheye.calibrate(
        object_points,
        image_points,
        image_size,
        K,
        D,
        flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW,
    )
    return rms, K, D


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    args = _build_arg_parser().parse_args()

    images_dir: Path = args.images.resolve()
    square_mm: float = args.square_mm
    cal_config_path: Path = args.config.resolve()
    sensor_config_path: Path = args.sensor_config.resolve()

    # -- Validate inputs -----------------------------------------------------
    if not images_dir.is_dir():
        logger.error(f"Images folder not found: {images_dir}")
        sys.exit(1)
    if not cal_config_path.exists():
        logger.error(f"Calibration config not found: {cal_config_path}")
        sys.exit(1)
    if not sensor_config_path.exists():
        logger.error(f"Sensor config not found: {sensor_config_path}")
        sys.exit(1)

    # -- Load board size from config -----------------------------------------
    board_size = _load_board_size(cal_config_path)
    logger.info(f"Board size: {board_size[0]}×{board_size[1]} inner corners")
    logger.info(f"Square size: {square_mm} mm")

    # -- Collect images ------------------------------------------------------
    image_paths = _collect_image_paths(images_dir)
    logger.info(f"Found {len(image_paths)} image(s) in {images_dir}")
    if len(image_paths) < 10:
        logger.error(
            f"Only {len(image_paths)} image(s) found — need at least 10 for a "
            f"reliable calibration. Collect more images and retry."
        )
        sys.exit(1)

    # -- Detect corners ------------------------------------------------------
    object_points, image_points, image_size = _detect_corners(image_paths, board_size)
    if len(object_points) < 10:
        logger.error(
            f"Only {len(object_points)} image(s) had detectable corners — need at "
            f"least 10. Recapture with better coverage or lighting."
        )
        sys.exit(1)

    # Scale object points from squares to millimetres
    scaled_object_points = [op * square_mm for op in object_points]

    # -- Calibrate -----------------------------------------------------------
    logger.info(f"Running fisheye.calibrate on {len(object_points)} image(s)...")
    rms, camera_matrix, dist_coeffs = _run_fisheye_calibration(
        scaled_object_points, image_points, image_size
    )

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    d = dist_coeffs.flatten()  # [k1, k2, k3, k4]

    print("\n--- Calibration results ---")
    print(f"  Images used   : {len(object_points)}")
    print(f"  Resolution    : {image_size[0]}×{image_size[1]} px")
    print(f"  RMS error     : {rms:.4f} px")
    print(f"  fx, fy        : {fx:.2f}, {fy:.2f}")
    print(f"  cx, cy        : {cx:.2f}, {cy:.2f}")
    print(f"  dist_coeffs   : {d.tolist()}  # [k1, k2, k3, k4]")
    print()

    # -- Validate ------------------------------------------------------------
    try:
        _validate(rms, camera_matrix, image_size)
        logger.info("Validation passed.")
    except ValueError as exc:
        logger.error(str(exc))
        sys.exit(1)

    # -- Write results -------------------------------------------------------
    _write_results(sensor_config_path, camera_matrix, dist_coeffs, image_size, rms)
    print(f"Results written to {sensor_config_path} under 'waveshare_rgb'.")


if __name__ == "__main__":
    main()
