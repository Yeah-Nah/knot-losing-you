"""YOLO-based person detection and tracking."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any

from loguru import logger

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray


def select_target_centroid(results: Any) -> tuple[float | None, float | None]:
    """Return the ``(u, v)`` centroid of the highest-confidence detection.

    Parameters
    ----------
    results : Any
        Ultralytics Results list returned by :meth:`ObjectDetection.run`.

    Returns
    -------
    tuple[float | None, float | None]
        Pixel coordinates ``(u, v)`` of the best detection centroid, or
        ``(None, None)`` when no detections are present.
    """
    if not results:
        return None, None
    boxes = results[0].boxes
    if boxes is None or len(boxes) == 0:
        return None, None
    best = int(boxes.conf.argmax())
    x1, y1, x2, y2 = boxes.xyxy[best].tolist()
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0


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
        from ultralytics import (
            YOLO,
        )  # deferred: slow import, only needed when inference enabled

        self._model = YOLO(str(model_path))
        self._conf = float(config.get("conf", 0.5))
        self._classes: list[int] = list(config.get("classes", [0]))  # type: ignore[arg-type]
        self._persist = bool(config.get("persist", True))
        self._verbose = bool(config.get("verbose", False))
        logger.debug(
            "ObjectDetection initialised (model={}, conf={}, classes={}).",
            model_path.name,
            self._conf,
            self._classes,
        )

    def run(self, frame: NDArray[np.uint8]) -> Any:
        """Run inference on a single frame.

        Parameters
        ----------
        frame : NDArray[np.uint8]
            BGR image frame from the camera.

        Returns
        -------
        list
            Ultralytics Results list containing detections and tracks.
        """
        common_kwargs = {
            "conf": self._conf,
            "classes": self._classes,
            "verbose": self._verbose,
        }

        if self._persist:
            # persist is a tracking argument — must use track() not predict()
            return self._model.track(frame, persist=True, **common_kwargs)

        return self._model(frame, **common_kwargs)
