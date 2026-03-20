"""OAK-D Lite camera access layer."""

from __future__ import annotations

from typing import TYPE_CHECKING

import depthai as dai
import numpy as np
from loguru import logger

if TYPE_CHECKING:
    from numpy.typing import NDArray


class CameraAccess:
    """Manages the OAK-D Lite camera connection and frame retrieval.

    Discovers the colour camera socket at runtime from the connected device,
    so no static socket configuration is needed.

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
        self._pipeline: dai.Pipeline | None = None
        self._video_queue: dai.DataOutputQueue | None = None
        self._colour_cam_name: str | None = None
        logger.debug("CameraAccess initialised.")

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _discover_colour_socket(self) -> dai.CameraFeatures:
        """Open a temporary device connection to find the colour camera socket.

        Returns
        -------
        dai.CameraFeatures
            Features of the first colour camera found.

        Raises
        ------
        RuntimeError
            If no OAK-D device is found, or if no colour camera is reported.
        """
        available = dai.Device.getAllAvailableDevices()
        if not available:
            raise RuntimeError("No OAK-D device found.")
        with dai.Device(available[0]) as device:
            features = list(device.getConnectedCameraFeatures())
        for feature in features:
            if dai.CameraType.COLOR in feature.supportedTypes:
                logger.debug(
                    f"Discovered colour camera: {feature.socket.name} "
                    f"({feature.sensorName})"
                )
                return feature
        raise RuntimeError("Connected OAK-D device reported no colour camera sensor.")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Open the camera connection and build the DepthAI pipeline.

        Raises
        ------
        RuntimeError
            If no OAK-D device is found or pipeline creation fails.
        """
        logger.info("Starting OAK-D colour camera...")
        try:
            colour_feature = self._discover_colour_socket()
            self._colour_cam_name = colour_feature.socket.name

            self._pipeline = dai.Pipeline()
            cam = self._pipeline.create(dai.node.Camera).build(colour_feature.socket)
            output = cam.requestOutput(self._resolution, fps=self._fps)
            self._video_queue = output.createOutputQueue(maxSize=16, blocking=False)
            self._pipeline.start()
        except RuntimeError:
            self._pipeline = None
            raise
        except Exception as exc:
            self._pipeline = None
            raise RuntimeError(f"Pipeline initialisation failed: {exc}") from exc

        logger.info(
            f"OAK-D camera started: socket={self._colour_cam_name}, "
            f"resolution={self._resolution}, fps={self._fps}."
        )

    def get_frame(self) -> NDArray[np.uint8] | None:
        """Retrieve the most recent colour frame from the queue.

        Returns
        -------
        NDArray[np.uint8] | None
            BGR frame as a numpy array, or ``None`` if no frame is ready
            or the camera has not been started.
        """
        if self._video_queue is None:
            return None
        packet = self._video_queue.tryGet()
        if packet is None:
            return None
        return packet.getCvFrame()  # type: ignore[no-any-return]

    def stop(self) -> None:
        """Close the camera connection and release resources."""
        logger.info("Stopping OAK-D camera...")
        if self._pipeline is not None:
            self._pipeline.stop()
        self._pipeline = None
        self._video_queue = None
        self._colour_cam_name = None
        logger.info("OAK-D camera stopped.")
