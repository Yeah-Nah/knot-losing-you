"""Step 2 validation script: open OAK-D colour feed in headless mode.

Test that there is a connection to the camera and that frames can
be retrieved at a reasonable rate. This is a smoke test for the
CameraAccess class, which is critical for the autonomous pipeline.

Usage
-----
    ugv-check-camera

    # Module form also works:
    python -m tools.check_camera_access
"""

from __future__ import annotations

import argparse
import sys
import time

from loguru import logger

from ugv_follower.perception.camera_access import CameraAccess


NONE_FRAME_BACKOFF_S = 0.02


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
    parser.add_argument(
        "--frame-timeout-s",
        type=float,
        default=5.0,
        help="Fail if no frame arrives for this many seconds (0 = disabled)",
    )
    args = parser.parse_args()

    camera = CameraAccess(fps=args.fps, resolution=(1920, 1080))

    try:
        camera.start()
        logger.success("start ✓")
        logger.info("Capturing frames headless — press Ctrl+C to quit.")

        frame_count = 0
        t0 = time.monotonic()
        last_frame_s = t0

        while True:
            frame = camera.get_frame()
            if frame is None:
                now = time.monotonic()
                if (
                    args.frame_timeout_s > 0.0
                    and (now - last_frame_s) >= args.frame_timeout_s
                ):
                    raise RuntimeError(
                        f"No frames received for {args.frame_timeout_s:.1f}s (camera timeout)."
                    )

                time.sleep(NONE_FRAME_BACKOFF_S)
                continue

            last_frame_s = time.monotonic()
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
        logger.exception(
            "Test failed: {}. Possible causes: camera not connected, insufficient "
            "USB/device permissions, or the device is already in use.",
            exc,
        )
        sys.exit(1)
    finally:
        camera.stop()
        logger.success("stop ✓")


if __name__ == "__main__":
    main()
