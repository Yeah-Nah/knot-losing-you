"""YOLO-based person detection and tracking."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any

from loguru import logger

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray


class ObjectDetection:
    """Wraps the YOLO model for person detection and tracking.

    Parameters
    ----------
    model_path : Path
        Path to the YOLO ``.pt`` or ``.onnx`` model file.
    config : dict[str, object]
        Model configuration dictionary from ``model_config.yaml``.
    """

    def __init__(self, model_path: Path, config: dict[str, object]) -> None:
        self._model_path = model_path
        self._config = config
        logger.debug("ObjectDetection initialised (stub).")

    def run(self, frame: NDArray[np.uint8]) -> Any:
        """Run inference on a single frame.

        Parameters
        ----------
        frame : NDArray[np.uint8]
            BGR image frame from the camera.

        Returns
        -------
        Results
            Ultralytics Results object containing detections and tracks.
        """
        raise NotImplementedError
