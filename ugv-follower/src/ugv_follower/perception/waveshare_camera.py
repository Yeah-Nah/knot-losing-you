"""Waveshare RGB pan-tilt camera access layer (V4L2 / cv2.VideoCapture)."""

from __future__ import annotations

from typing import TYPE_CHECKING

import cv2
from loguru import logger

from ..utils.camera_preflight import ensure_camera_device_available

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray


class WaveshareCamera:
    """cv2.VideoCapture wrapper for the Waveshare RGB pan-tilt camera.

    Parameters
    ----------
    device_index : int
        V4L2 device index (maps to ``/dev/video<n>``).
    width : int
        Requested frame width in pixels.
    height : int
        Requested frame height in pixels.
    fps : int
        Requested capture frame rate.
    """

    def __init__(
        self,
        device_index: int,
        width: int,
        height: int,
        fps: int,
    ) -> None:
        self._device_index = device_index
        self._width = width
        self._height = height
        self._fps = fps
        self._cap: cv2.VideoCapture | None = None
        logger.debug(
            "WaveshareCamera initialised (device={}, {}x{}, {}fps).",
            device_index,
            width,
            height,
            fps,
        )

    def start(self) -> None:
        """Open the camera device and run preflight checks.

        Raises
        ------
        RuntimeError
            If the device is busy, missing, or cannot be opened.
        """
        device_path = f"/dev/video{self._device_index}"
        ensure_camera_device_available(device_path)

        logger.info("Opening Waveshare camera (device={})...", self._device_index)
        cap = cv2.VideoCapture(self._device_index, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(cv2.CAP_PROP_FPS, self._fps)
        cap.set(
            cv2.CAP_PROP_BUFFERSIZE, 1
        )  # Discard all but the latest frame; prevents stale-frame lag
        if not cap.isOpened():
            raise RuntimeError(
                f"Could not open Waveshare camera at device index {self._device_index}."
            )
        self._cap = cap
        logger.info(
            "Waveshare camera started (device={}, {}x{}, {}fps).",
            self._device_index,
            self._width,
            self._height,
            self._fps,
        )

    def get_frame(self) -> NDArray[np.uint8] | None:
        """Read the next frame from the camera.

        Returns
        -------
        NDArray[np.uint8] | None
            BGR frame, or ``None`` if capture is not started or read fails.
        """
        if self._cap is None:
            return None
        ok, frame = self._cap.read()
        if not ok:
            logger.warning("WaveshareCamera.read() failed.")
            return None
        return frame  # type: ignore[return-value]

    def stop(self) -> None:
        """Release the camera device."""
        logger.info("Stopping Waveshare camera...")
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        logger.info("Waveshare camera stopped.")
