"""Step 2 validation script: open OAK-D colour feed in a live window.

Run with X forwarding enabled (ssh -X pi@<ip>) so the window can display
on your dev machine.

Usage
-----
    python -m tests.test_camera_access

    # Lower resolution for faster X-forwarding:
    python -m tests.test_camera_access --scale 0.25

    # Headless: print frame stats only, no window (useful if display is unavailable):
    python -m tests.test_camera_access --no-display
"""

from __future__ import annotations

import argparse
import sys
import time

from loguru import logger

from ugv_follower.perception.camera_access import CameraAccess


def main() -> None:
    parser = argparse.ArgumentParser(description="CameraAccess headless smoke test.")
    parser.add_argument("--fps", type=int, default=30, help="Target camera frame rate")
    parser.add_argument(
        "--log-every",
        type=int,
        default=30,
        help="Log one status line every N frames",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Stop after N frames (0 = run until Ctrl+C)",
    )
    args = parser.parse_args()

    camera = CameraAccess(fps=args.fps, resolution=(1920, 1080))

    try:
        camera.start()
        logger.success("start ✓")
        logger.info("Capturing frames headless — press Ctrl+C to quit.")

        frame_count = 0
        t0 = time.monotonic()

        while True:
            frame = camera.get_frame()
            if frame is None:
                continue

            frame_count += 1

            if frame_count % max(1, args.log_every) == 0:
                elapsed = max(1e-6, time.monotonic() - t0)
                logger.info(
                    f"frame {frame_count} ✓ shape={frame.shape} dtype={frame.dtype} "
                    f"avg_fps={frame_count / elapsed:.1f}"
                )

            if args.max_frames > 0 and frame_count >= args.max_frames:
                break

    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
    except Exception as exc:
        logger.error(f"Test failed: {exc}")
        sys.exit(1)
    finally:
        camera.stop()
        logger.success("stop ✓")
