"""Step 2 validation script: open OAK-D colour feed in a live window.

Run with X forwarding enabled (ssh -X pi@<ip>) so the window can display
on your dev machine.

Usage
-----
    python test_camera_access.py

    # Lower resolution for faster X-forwarding:
    python test_camera_access.py --scale 0.25

    # Headless: print frame stats only, no window (useful if display is unavailable):
    python test_camera_access.py --no-display
"""

from __future__ import annotations

import argparse
import sys

import cv2
from loguru import logger

from src.perception.camera_access import CameraAccess


def main() -> None:
    parser = argparse.ArgumentParser(description="CameraAccess step-2 smoke test.")
    parser.add_argument(
        "--scale",
        type=float,
        default=0.5,
        help="Scale factor applied before display (default: 0.5). "
             "Reduce for better X-forwarding performance.",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Target camera frame rate (default: 30)",
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Skip cv2.imshow — just print frame stats. Useful for headless testing.",
    )
    args = parser.parse_args()

    camera = CameraAccess(fps=args.fps, resolution=(1920, 1080))

    try:
        camera.start()
        logger.success("start ✓")

        logger.info("Capturing frames — press 'q' to quit.")
        frame_count = 0

        while True:
            frame = camera.get_frame()
            if frame is None:
                continue

            frame_count += 1

            if args.no_display:
                if frame_count % 30 == 0:
                    logger.info(
                        f"frame {frame_count} received ✓  shape={frame.shape}  "
                        f"dtype={frame.dtype}"
                    )
            else:
                display = cv2.resize(frame, (0, 0), fx=args.scale, fy=args.scale)
                cv2.imshow("OAK-D Colour Feed — press q to quit", display)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
    except Exception as exc:
        logger.error(f"Test failed: {exc}")
        sys.exit(1)
    finally:
        camera.stop()
        logger.success("stop ✓")
        if not args.no_display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
