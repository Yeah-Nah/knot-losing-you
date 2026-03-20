"""OAK-D Lite camera access layer."""

from __future__ import annotations

from loguru import logger


class CameraAccess:
    """Manages the OAK-D Lite camera connection and frame retrieval.

    Parameters
    ----------
    fps : int
        Target frames per second for the colour camera output.
    resolution : tuple[int, int]
        Output resolution as (width, height).
    """

    def __init__(
        self, fps: int = 30, resolution: tuple[int, int] = (1920, 1080)
    ) -> None:
        self._fps = fps
        self._resolution = resolution
        logger.debug("CameraAccess initialised (stub).")

    def start(self) -> None:
        """Open the camera connection and build the DepthAI pipeline."""
        raise NotImplementedError

    def stop(self) -> None:
        """Close the camera connection and release resources."""
        raise NotImplementedError
