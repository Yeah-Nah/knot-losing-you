"""Pipeline orchestration for ugv-follower.

This module coordinates perception, inference, and control
for the autonomous UGV follower.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from loguru import logger

if TYPE_CHECKING:
    from .settings import Settings


class Pipeline:
    """Orchestrates perception, inference, and UGV control.

    Parameters
    ----------
    settings : Settings
        Fully resolved and validated settings instance.
    """

    def __init__(self, settings: Settings) -> None:
        self._settings = settings
        logger.info("Pipeline initialised.")

    def run(self) -> None:
        """Start the pipeline and enter the main processing loop."""
        logger.info("Starting pipeline...")
        try:
            self._main_loop()
        except KeyboardInterrupt:
            logger.info("Pipeline interrupted by user.")
        finally:
            self._shutdown()

    def _main_loop(self) -> None:
        """Main processing loop — to be implemented."""
        logger.warning("Pipeline main loop not yet implemented.")

    def _shutdown(self) -> None:
        """Release resources on exit."""
        logger.info("Pipeline shut down.")
