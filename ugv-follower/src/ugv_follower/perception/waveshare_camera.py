"""Waveshare RGB pan-tilt camera access layer (V4L2 / cv2.VideoCapture).

A background grabber thread drains the camera at its native frame rate and
keeps only the newest frame plus its capture timestamp.  Consumers call
:meth:`WaveshareCamera.get_frame` to obtain that frame without blocking on
V4L2 I/O, so a slow control loop always processes the freshest image rather
than a buffer queued up since the previous read.

Concurrency guardrails
----------------------
- Threading coordination  : one daemon grabber thread with an Event stop
                            signal and bounded join.
- Shared-state sync       : threading.Lock protects the latest-frame reference;
                            held only for the atomic pointer swap, not during I/O.
- Race conditions         : CameraFrame is frozen — the grabber replaces the
                            reference atomically; readers use only their copy.
                            The image array is shared and must not be mutated.
- Shutdown order          : the capture is released only after the grabber
                            thread has exited; releasing during an in-flight
                            read() can crash the native backend.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, cast

import cv2
from loguru import logger

from ..utils.camera_preflight import ensure_camera_device_available

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray


_THREAD_JOIN_TIMEOUT_S: float = 2.0  # matches PanTelemetryPoller.stop()
_READ_FAILURE_BACKOFF_S: float = 0.01  # avoid busy-spin when read() fails


@dataclass(frozen=True)
class CameraFrame:
    """One captured camera frame with its capture timestamp.

    Parameters
    ----------
    image : NDArray[np.uint8]
        BGR frame.  Shared between consumers — treat as read-only.
    capture_time_monotonic : float
        ``time.monotonic()`` timestamp taken immediately after ``read()``
        returned this frame.
    seq : int
        Monotonically increasing counter; advances on every successful read.

    Notes
    -----
    ``age_s`` is computed at call time so it reflects the true age of the
    frame, not the age of this object.
    """

    image: NDArray[np.uint8]
    capture_time_monotonic: float
    seq: int

    @property
    def age_s(self) -> float:
        """Seconds elapsed since the frame was captured."""
        return time.monotonic() - self.capture_time_monotonic


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

        # Shared state — protected by _lock
        self._latest: CameraFrame | None = None
        self._seq: int = 0
        self._lock = threading.Lock()

        # Grabber-thread-only state
        self._read_failing: bool = False

        # Thread lifecycle
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

        logger.debug(
            "WaveshareCamera initialised (device={}, {}x{}, {}fps).",
            device_index,
            width,
            height,
            fps,
        )

    def start(self) -> None:
        """Open the camera device, run preflight checks, and start grabbing.

        Raises
        ------
        RuntimeError
            If the camera is already running, or the device is busy, missing,
            or cannot be opened.
        """
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("WaveshareCamera is already running.")
        self._cap = self._open_capture()

        with self._lock:
            self._latest = None
            self._seq = 0
        self._read_failing = False
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._grab_loop,
            name="WaveshareCameraGrabber",
            daemon=True,
        )
        self._thread.start()
        logger.info(
            "Waveshare camera started (device={}, {}x{}, {}fps).",
            self._device_index,
            self._width,
            self._height,
            self._fps,
        )

    def get_frame(self) -> CameraFrame | None:
        """Return the newest captured frame without blocking on I/O.

        Returns
        -------
        CameraFrame | None
            Latest frame with its capture timestamp, or ``None`` before
            ``start()``, after ``stop()``, or until the first frame arrives.
        """
        with self._lock:
            return self._latest

    def stop(self) -> None:
        """Stop the grabber thread and release the camera device."""
        logger.info("Stopping Waveshare camera...")
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=_THREAD_JOIN_TIMEOUT_S)
            if self._thread.is_alive():
                # Releasing during an in-flight read() can crash the backend;
                # leave the device to be closed at process exit instead.
                logger.warning(
                    "WaveshareCamera grabber did not exit within {} s; "
                    "skipping capture release.",
                    _THREAD_JOIN_TIMEOUT_S,
                )
                self._cap = None
            self._thread = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        with self._lock:
            self._latest = None
        logger.info("Waveshare camera stopped.")

    def _open_capture(self) -> cv2.VideoCapture:
        """Run device preflight then open and configure the V4L2 capture.

        Returns
        -------
        cv2.VideoCapture
            Opened capture configured with the requested size and rate.

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
        # Reduce driver queue depth; frame freshness comes from the grabber
        # thread draining the camera at its native rate.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(
                f"Could not open Waveshare camera at device index {self._device_index}."
            )
        return cap

    def _grab_loop(self) -> None:
        """Grabber thread — read frames continuously until stopped."""
        logger.debug("WaveshareCamera grabber thread running.")
        while not self._stop_event.is_set():
            self._capture_once()
        logger.debug("WaveshareCamera grabber thread exiting.")

    def _capture_once(self) -> bool:
        """Read one frame and publish it as the latest snapshot.

        On failure the previous frame is left in place so its ``age_s``
        grows naturally.

        Returns
        -------
        bool
            ``True`` if a frame was read and published, ``False`` otherwise.
        """
        if self._cap is None:
            return False
        ok, image = self._cap.read()
        capture_time = time.monotonic()
        if not ok:
            if not self._read_failing:
                logger.warning("WaveshareCamera.read() failed; retrying.")
                self._read_failing = True
            self._stop_event.wait(_READ_FAILURE_BACKOFF_S)
            return False
        if self._read_failing:
            logger.info("WaveshareCamera.read() recovered.")
            self._read_failing = False
        with self._lock:
            self._seq += 1
            self._latest = CameraFrame(
                # cv2 stubs type read() output as Any; V4L2 BGR frames are uint8.
                image=cast("NDArray[np.uint8]", image),
                capture_time_monotonic=capture_time,
                seq=self._seq,
            )
        return True
