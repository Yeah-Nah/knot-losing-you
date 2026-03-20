"""Step 1 validation script: connect → drive forward 1 second → stop → disconnect.

Run this with the rover elevated so the wheels are clear of the ground.

Usage
-----
    python test_ugv_controller.py

    # Override the default port (Pi 4B uses /dev/serial0):
    python test_ugv_controller.py --port /dev/serial0
"""

from __future__ import annotations

import argparse
import sys
import time

from loguru import logger

from src.control.ugv_controller import UGVController


def main() -> None:
    parser = argparse.ArgumentParser(description="UGVController step-1 smoke test.")
    parser.add_argument(
        "--port",
        default="/dev/ttyAMA0",
        help="Serial port for the UGV sub-controller (default: /dev/ttyAMA0 for Pi 5; "
             "use /dev/serial0 for Pi 4B)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.2,
        help="Forward speed in m/s (default: 0.2)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1.0,
        help="Duration to drive forward in seconds (default: 1.0)",
    )
    args = parser.parse_args()

    controller = UGVController(port=args.port)

    try:
        controller.connect()
        logger.success("connect ✓")

        logger.info(f"Driving forward at {args.speed} m/s for {args.duration} s...")
        controller.move(linear=args.speed, angular=0.0)
        logger.success("move sent ✓")

        time.sleep(args.duration)

        controller.stop()
        logger.success("stop sent ✓")

        controller.disconnect()
        logger.success("disconnect ✓")

    except Exception as exc:
        logger.error(f"Test failed: {exc}")
        # Attempt emergency stop before exiting
        try:
            controller.stop()
        except Exception:
            pass
        sys.exit(1)


if __name__ == "__main__":
    main()
