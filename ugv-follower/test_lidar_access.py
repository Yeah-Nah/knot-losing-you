"""Step 3 validation script: read packets from the LDRobot D500 LiDAR.

Prints distance and angle readings to the console.  No display needed.

Usage
-----
    python test_lidar_access.py
    python test_lidar_access.py --port /dev/ttyUSB0 --count 10
"""

from __future__ import annotations

import argparse
import sys

from loguru import logger

from src.perception.lidar_access import LidarAccess


def main() -> None:
    parser = argparse.ArgumentParser(description="LidarAccess step-3 smoke test.")
    parser.add_argument(
        "--port",
        default="/dev/ttyUSB0",
        help="Serial port for the LiDAR (default: /dev/ttyUSB0)",
    )
    parser.add_argument(
        "--baud-rate",
        type=int,
        default=230400,
        help="Serial baud rate (default: 230400)",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=5,
        help="Number of packets to read before exiting (default: 5)",
    )
    args = parser.parse_args()

    lidar = LidarAccess(port=args.port, baud_rate=args.baud_rate)

    try:
        lidar.start()
        logger.success("start ✓")

        received = 0
        skipped = 0

        while received < args.count:
            points = lidar.get_scan()

            if points is None:
                skipped += 1
                if skipped > 20:
                    logger.error(
                        "Too many consecutive failures — check port, baud rate, "
                        "and that the device is powered on."
                    )
                    sys.exit(1)
                continue

            skipped = 0
            received += 1

            distances = [p["distance"] for p in points if p["distance"] > 0]
            min_d = min(distances) if distances else 0
            max_d = max(distances) if distances else 0
            mean_d = int(sum(distances) / len(distances)) if distances else 0

            logger.info(
                f"Packet {received}/{args.count} — "
                f"{len(points)} points | "
                f"valid={len(distances)} | "
                f"min={min_d} mm  max={max_d} mm  mean={mean_d} mm | "
                f"angles {points[0]['angle']:.1f}°→{points[-1]['angle']:.1f}°"
            )

            # Print each point on the first packet so you can see the raw data
            if received == 1:
                for p in points:
                    logger.debug(
                        f"  angle={p['angle']:6.2f}°  "
                        f"dist={p['distance']:5d} mm  "
                        f"intensity={p['intensity']}"
                    )

        logger.success(f"Received {received} packets ✓")

    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
    except Exception as exc:
        logger.error(f"Test failed: {exc}")
        sys.exit(1)
    finally:
        lidar.stop()
        logger.success("stop ✓")


if __name__ == "__main__":
    main()
