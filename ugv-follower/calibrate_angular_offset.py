"""Angular offset calibration between the LDRobot D500 LiDAR and the Waveshare pan-tilt.

Measures ``delta_offset`` — the signed yaw angle (degrees) between the LiDAR's 0° forward
axis and the pan-tilt camera's mechanical zero — and writes it to ``sensor_config.yaml``
under the ``extrinsic`` key.

Must be run on the Raspberry Pi with the LiDAR and UGV rover connected.
Intrinsic calibration (``calibrate_waveshare_camera.py``) must have been completed first.

How to run
----------
::

    python calibrate_angular_offset.py --distance 2.0

Arguments
---------
--distance       Approximate distance to the calibration target in metres (required).
--distance-tol   Half-width of the LiDAR range filter in metres (default: 0.2).
                 Only returns within [distance − tolerance, distance + tolerance] are kept.
--duration       Seconds to accumulate LiDAR data after the target is aligned (default: 5.0).
--camera-index   OpenCV VideoCapture index for the Waveshare RGB camera (default: 0).
--sensor-config  Path to sensor_config.yaml to read and write (default:
                 configs/sensor_config.yaml relative to this script).

Hardware setup
--------------
1. Mount the UGV on a flat surface with clear space ahead (≥ 2 m recommended).
2. Connect the LDRobot D500 to the port listed under ``lidar.port`` in sensor_config.yaml.
3. Connect the UGV rover sub-controller to the port listed under ``ugv.port``.
4. Connect the Waveshare RGB camera (USB) — note its VideoCapture index.
5. Power on the rover.

Calibration procedure
---------------------
1. Run the script with the desired ``--distance`` value.
2. The rover commands the pan-tilt to its mechanical zero (0°, 0°) and waits for the
   servo to settle.
3. A live camera feed opens with a vertical green guide line drawn at column ``cx``
   (the principal point from intrinsic calibration — not the pixel-count midpoint).
4. Place a flat-faced cardboard box (≈ 20 cm wide) roughly ``--distance`` metres directly
   in front of the rover.  Physically slide it left or right until its face is bisected
   by the green line.
5. Press **SPACE** (or **ENTER**) to confirm the target is aligned.
   Press **Q** to abort without saving.
6. The script closes the camera window and accumulates LiDAR returns for ``--duration``
   seconds.  It keeps only returns in the forward ±30° arc at the expected distance.
7. The angular centroid of the cluster is computed as the median of the signed angles,
   avoiding wrap-around artefacts near 0°/360°.
8. A summary is printed.  Type **y** and press ENTER to write the result to
   sensor_config.yaml; any other input exits without saving.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from loguru import logger

_SCRIPT_DIR = Path(__file__).resolve().parent
_DEFAULT_SENSOR_CONFIG = _SCRIPT_DIR / "configs" / "sensor_config.yaml"

# Add the ugv-follower directory to sys.path so src.* imports resolve when
# the package is not installed (e.g. running directly from the repo).
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from src.control.ugv_controller import UGVController  # noqa: E402
from src.perception.lidar_access import LidarAccess, LidarPoint  # noqa: E402

# Half-width of the forward arc that is searched for LiDAR returns (degrees).
# Centred on 0°; covers [0, _FORWARD_ARC_DEG] and [360 - _FORWARD_ARC_DEG, 360).
_FORWARD_ARC_DEG: float = 30.0

# Minimum cluster size below which the result is flagged as noisy.
_MIN_CLUSTER_POINTS: int = 20

# Seconds to wait after set_pan_tilt(0, 0) for the servo to settle.
_SERVO_SETTLE_S: float = 1.5

# OpenCV window name for the guide overlay.
_WINDOW_NAME = "Angular Offset Calibration — press SPACE to confirm, Q to quit"


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------


def _extract_cx(cfg: dict) -> float:
    """Return the horizontal principal point ``cx`` from ``waveshare_rgb.camera_matrix``.

    Parameters
    ----------
    cfg:
        Parsed sensor_config.yaml dict.

    Returns
    -------
    float
        ``cx`` in pixels.

    Raises
    ------
    SystemExit
        If ``camera_matrix`` is ``null`` (intrinsic calibration not yet run).
    """
    matrix = cfg.get("waveshare_rgb", {}).get("camera_matrix")
    if matrix is None:
        logger.error(
            "waveshare_rgb.camera_matrix is null in sensor_config.yaml. "
            "Run calibrate_waveshare_camera.py first."
        )
        sys.exit(1)
    # camera_matrix is stored row-major: [[fx,0,cx],[0,fy,cy],[0,0,1]]
    cx: float = float(matrix[0][2])
    return cx


def _in_forward_arc(angle_deg: float, arc: float = _FORWARD_ARC_DEG) -> bool:
    """Return True if *angle_deg* is within ±arc degrees of 0° (wrap-safe).

    Accepts angles in [0, arc] or [360 − arc, 360) without any modular
    arithmetic that could fail at the boundary.
    """
    return angle_deg <= arc or angle_deg >= (360.0 - arc)


def _to_signed(angle_deg: float) -> float:
    """Convert an angle in [0, 360) to a signed angle in (−180, +180].

    This prevents the wrap-around artefact that occurs when averaging angles
    straddling the 0°/360° boundary (e.g. 359° and 1° should average to 0°,
    not 180°).
    """
    return angle_deg if angle_deg <= 180.0 else angle_deg - 360.0


# ---------------------------------------------------------------------------
# Hardware interaction
# ---------------------------------------------------------------------------


def _accumulate_lidar(
    lidar: LidarAccess,
    duration_s: float,
    dist_min_mm: float,
    dist_max_mm: float,
) -> list[float]:
    """Collect LiDAR returns from the forward arc within the distance window.

    Runs for *duration_s* seconds, calling ``lidar.get_scan()`` in a tight
    loop.  Each returned packet yields up to 12 points; only those in the
    forward arc and within the range window are kept.

    Parameters
    ----------
    lidar:
        Started ``LidarAccess`` instance.
    duration_s:
        How long to accumulate (seconds).
    dist_min_mm:
        Minimum accepted distance in millimetres.
    dist_max_mm:
        Maximum accepted distance in millimetres.

    Returns
    -------
    list[float]
        Signed angles (degrees, (−180, +180]) of all accepted returns.
    """
    signed_angles: list[float] = []
    deadline = time.monotonic() + duration_s

    while time.monotonic() < deadline:
        packet: list[LidarPoint] | None = lidar.get_scan()
        if packet is None:
            continue
        for pt in packet:
            dist = pt["distance"]
            angle = pt["angle"]
            if dist == 0:
                continue  # sentinel: out of range
            if not _in_forward_arc(angle):
                continue
            if not (dist_min_mm <= dist <= dist_max_mm):
                continue
            signed_angles.append(_to_signed(angle))

    return signed_angles


# ---------------------------------------------------------------------------
# Guide overlay
# ---------------------------------------------------------------------------


def _run_guide_overlay(cap: cv2.VideoCapture, cx: float) -> bool:
    """Display the live camera feed with a vertical guide line at column *cx*.

    Blocks until the user presses SPACE/ENTER (returns True) or Q (returns False).

    Parameters
    ----------
    cap:
        Opened ``cv2.VideoCapture`` handle.
    cx:
        Horizontal principal point column in pixels.

    Returns
    -------
    bool
        ``True`` if the user confirmed target alignment; ``False`` if they quit.
    """
    cx_col = int(round(cx))
    cv2.namedWindow(_WINDOW_NAME, cv2.WINDOW_NORMAL)

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            logger.warning("Camera returned no frame — check --camera-index.")
            time.sleep(0.05)
            continue

        h = frame.shape[0]
        # Draw full-height vertical guide line at cx
        cv2.line(frame, (cx_col, 0), (cx_col, h - 1), (0, 255, 0), 2)

        # Instruction overlays
        cv2.putText(
            frame,
            "Centre target on green line, then press SPACE",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f"cx = {cx:.1f} px   |   Q = quit",
            (10, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

        cv2.imshow(_WINDOW_NAME, frame)
        key = cv2.waitKey(30) & 0xFF

        if key in (ord(" "), 13):  # SPACE or ENTER
            return True
        if key in (ord("q"), ord("Q"), 27):  # Q or ESC
            return False


# ---------------------------------------------------------------------------
# YAML write
# ---------------------------------------------------------------------------


def _write_results(
    sensor_config_path: Path,
    delta_offset_deg: float,
    target_distance_m: float,
    n_lidar_points: int,
) -> None:
    """Merge the calibration result into sensor_config.yaml under ``extrinsic``.

    Reads the existing file, updates (or creates) the ``extrinsic`` key, and
    re-serialises the full configuration.  All other top-level keys are
    preserved; comments and original formatting are not.

    Parameters
    ----------
    sensor_config_path:
        Path to ``sensor_config.yaml``.
    delta_offset_deg:
        Measured angular offset in signed degrees (−180, +180].
    target_distance_m:
        Target distance used during calibration, in metres.
    n_lidar_points:
        Number of LiDAR cluster points used to compute the median.
    """
    with sensor_config_path.open() as f:
        config: dict = yaml.safe_load(f) or {}

    config["extrinsic"] = {
        "lidar_to_pantilt_offset_deg": round(float(delta_offset_deg), 4),
        "calibration_method": "angular_offset",
        "target_distance_m": round(float(target_distance_m), 3),
        "n_lidar_points": int(n_lidar_points),
    }

    with sensor_config_path.open("w") as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    logger.info(f"Results written to {sensor_config_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _build_arg_parser() -> argparse.ArgumentParser:
    """Build and return the CLI argument parser."""
    parser = argparse.ArgumentParser(
        description=(
            "Measure the yaw angular offset between the LiDAR forward axis "
            "and the pan-tilt mechanical zero."
        )
    )
    parser.add_argument(
        "--distance",
        required=True,
        type=float,
        metavar="METRES",
        help="Approximate distance to the calibration target in metres.",
    )
    parser.add_argument(
        "--distance-tol",
        type=float,
        default=0.2,
        metavar="METRES",
        help="Half-width of the LiDAR range filter in metres (default: 0.2).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        metavar="SECONDS",
        help="Seconds to accumulate LiDAR data after target alignment (default: 5.0).",
    )
    parser.add_argument(
        "--camera-index",
        type=int,
        default=0,
        metavar="INT",
        help="OpenCV VideoCapture index for the Waveshare RGB camera (default: 0).",
    )
    parser.add_argument(
        "--sensor-config",
        type=Path,
        default=_DEFAULT_SENSOR_CONFIG,
        metavar="PATH",
        help=f"Path to sensor_config.yaml (default: {_DEFAULT_SENSOR_CONFIG}).",
    )
    return parser


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    args = _build_arg_parser().parse_args()

    target_distance_m: float = args.distance
    distance_tol_m: float = args.distance_tol
    duration_s: float = args.duration
    camera_index: int = args.camera_index
    sensor_config_path: Path = args.sensor_config.resolve()

    # -- Validate inputs -------------------------------------------------------
    if target_distance_m <= 0:
        logger.error("--distance must be a positive number.")
        sys.exit(1)
    if target_distance_m < 1.0:
        logger.warning(
            f"Target distance {target_distance_m} m is less than 1 m. "
            "Sensor translation offset may introduce significant bearing error. "
            "Consider using a distance ≥ 1.5 m."
        )
    if not sensor_config_path.exists():
        logger.error(f"Sensor config not found: {sensor_config_path}")
        sys.exit(1)

    # -- Load config -----------------------------------------------------------
    with sensor_config_path.open() as f:
        cfg: dict = yaml.safe_load(f) or {}

    cx = _extract_cx(cfg)
    logger.info(f"Loaded cx = {cx:.2f} px from {sensor_config_path}")

    lidar_port: str = cfg["lidar"]["port"]
    lidar_baud: int = cfg["lidar"]["baud_rate"]
    ugv_port: str = cfg["ugv"]["port"]
    ugv_baud: int = cfg["ugv"]["baud_rate"]
    chassis_main: int = cfg["ugv"]["chassis_main"]
    chassis_module: int = cfg["ugv"]["chassis_module"]
    track_width: float = cfg["ugv"]["track_width"]

    dist_min_mm = (target_distance_m - distance_tol_m) * 1000.0
    dist_max_mm = (target_distance_m + distance_tol_m) * 1000.0

    logger.info(
        f"Target distance: {target_distance_m} m  "
        f"(range filter: [{dist_min_mm:.0f}, {dist_max_mm:.0f}] mm)"
    )

    # -- Initialise hardware ---------------------------------------------------
    ugv = UGVController(
        port=ugv_port,
        baud_rate=ugv_baud,
        chassis_main=chassis_main,
        chassis_module=chassis_module,
        track_width=track_width,
    )
    lidar = LidarAccess(port=lidar_port, baud_rate=lidar_baud)
    cap: cv2.VideoCapture | None = None

    try:
        # -- Step 2: Connect hardware and zero the pan-tilt --------------------
        ugv.connect()
        ugv.set_pan_tilt(0.0, 0.0)
        logger.info(f"Pan-tilt commanded to (0°, 0°). Waiting {_SERVO_SETTLE_S} s to settle...")
        time.sleep(_SERVO_SETTLE_S)

        lidar.start()

        # -- Step 3: Camera guide overlay --------------------------------------
        cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        if not cap.isOpened():
            logger.error(
                f"Could not open camera index {camera_index}. "
                "Try a different --camera-index."
            )
            sys.exit(1)

        logger.info("Opening camera guide overlay. Centre the target on the green line.")
        confirmed = _run_guide_overlay(cap, cx)
        cv2.destroyAllWindows()
        cap.release()
        cap = None

        if not confirmed:
            logger.info("Calibration aborted by user.")
            return

        # -- Step 4: LiDAR accumulation ----------------------------------------
        logger.info(f"Accumulating LiDAR for {duration_s} s...")
        signed_angles = _accumulate_lidar(lidar, duration_s, dist_min_mm, dist_max_mm)

        # -- Step 6: Centroid and validation -----------------------------------
        n_pts = len(signed_angles)
        if n_pts == 0:
            logger.error(
                "No LiDAR returns matched the forward arc and distance filter. "
                "Check that the target is visible to the LiDAR and within "
                f"{target_distance_m} ± {distance_tol_m} m."
            )
            sys.exit(1)

        if n_pts < _MIN_CLUSTER_POINTS:
            logger.warning(
                f"Only {n_pts} LiDAR point(s) in cluster — fewer than {_MIN_CLUSTER_POINTS}. "
                "Result may be noisy. Consider increasing --duration or moving the target closer."
            )

        delta_offset_deg = float(np.median(signed_angles))

        # -- Step 7: Summary and confirmation ----------------------------------
        print()
        print("Angular offset calibration result")
        print(f"  delta_offset      :  {delta_offset_deg:+.4f}°")
        print(f"  Cluster points    :  {n_pts}")
        print(f"  Target distance   :  {target_distance_m} m")
        print(f"  Range filter      :  [{target_distance_m - distance_tol_m:.2f}, "
              f"{target_distance_m + distance_tol_m:.2f}] m")
        print()

        answer = input("Save result to sensor_config.yaml? [y/N]: ").strip().lower()
        if answer != "y":
            logger.info("Result not saved.")
            return

        # -- Step 8: Write YAML ------------------------------------------------
        _write_results(sensor_config_path, delta_offset_deg, target_distance_m, n_pts)
        print(f"Result written to {sensor_config_path} under 'extrinsic'.")

    finally:
        # -- Step 9: Teardown --------------------------------------------------
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        lidar.stop()
        ugv.disconnect()


if __name__ == "__main__":
    main()
