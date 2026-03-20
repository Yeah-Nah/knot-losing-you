"""Entry point for the ugv-follower pipeline.

Connects to the OAK-D Lite camera and LiDAR, optionally runs YOLO inference
for person detection, and issues drive commands to the Waveshare UGV Rover.

Usage
-----
    python run_follower.py
    python run_follower.py --pipeline-config configs/pipeline_config.yaml
    python run_follower.py --model-config configs/model_config.yaml
    python run_follower.py --sensor-config configs/sensor_config.yaml
"""

from __future__ import annotations

import argparse
import sys

from loguru import logger
from src.pipeline import Pipeline
from src.settings import Settings


def main() -> None:
    """Parse arguments, load settings, and run the pipeline."""
    parser = argparse.ArgumentParser(description="UGV follower pipeline.")
    parser.add_argument(
        "--pipeline-config",
        default="configs/pipeline_config.yaml",
        help="Path to the pipeline config YAML",
    )
    parser.add_argument(
        "--model-config",
        default="configs/model_config.yaml",
        help="Path to the model config YAML",
    )
    parser.add_argument(
        "--sensor-config",
        default="configs/sensor_config.yaml",
        help="Path to the sensor config YAML",
    )
    args = parser.parse_args()

    logger.info("Loading configuration...")
    try:
        settings = Settings(
            pipeline_config_path=args.pipeline_config,
            model_config_path=args.model_config,
            sensor_config_path=args.sensor_config,
        )
    except (FileNotFoundError, OSError, ValueError) as exc:
        logger.error(f"Configuration error: {exc}")
        sys.exit(1)

    pipeline = Pipeline(settings)
    pipeline.run()


if __name__ == "__main__":
    main()
