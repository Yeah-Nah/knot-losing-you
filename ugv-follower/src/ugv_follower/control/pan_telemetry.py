"""Pan servo telemetry polling — decoupled from the vision loop.

Provides an immutable snapshot type and a two-thread background poller:

- Cadence thread  : sleeps for exactly ``poll_interval_*_s`` then signals
                    the serial worker.  Never blocks on I/O, so the polling
                    interval is independent of serial timing.
- Serial worker   : waits for the cadence trigger then performs a single
                    blocking ``query_pan_deg()`` call and publishes the
                    result.

The vision/control loop reads the latest snapshot once per cycle via
:meth:`PanTelemetryPoller.get_snapshot` without touching serial directly.

Concurrency guardrails
----------------------
- Threading coordination  : two daemon threads (cadence + serial worker) each
                            with an Event stop signal and bounded join.
- Trigger handoff         : threading.Event — idempotent set; worker clears
                            after pickup; cadence fires and forgets.
- Shared-state sync       : threading.Lock protects the snapshot reference;
                            held only for the atomic pointer swap, not during I/O.
- Race conditions         : PanTelemetrySnapshot is frozen — the writer replaces
                            the reference atomically; the reader copies the ref
                            before releasing the lock and uses only that copy.
- Poll thread idle load   : set_tracking_mode(False) reduces cadence to idle Hz
                            when control is idle (MANUAL mode or estop active).
- Late reply handling     : publish only if snap.seq > self._snapshot.seq; stale
                            responses from the serial bus cannot overwrite newer
                            state.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal

from loguru import logger

if TYPE_CHECKING:
    from .ugv_controller import UGVController

TelemetryStatus = Literal["initialising", "fresh", "stale", "expired"]


@dataclass(frozen=True)
class PanTelemetrySnapshot:
    """One coherent pan servo telemetry sample.

    Parameters
    ----------
    pan_deg : float
        Measured pan servo angle in degrees.
    sample_time_monotonic : float
        ``time.monotonic()`` timestamp at which the hardware sample was received.
    seq : int
        Monotonically increasing sequence counter; advances on each valid poll.
    valid : bool
        ``True`` when this snapshot carries a usable measurement (status is
        ``"fresh"``).  ``False`` for stale, expired, and initialising snapshots.
    status : TelemetryStatus
        Freshness classification at the moment this snapshot was produced or
        re-classified by ``_classify()``.

    Notes
    -----
    ``age_s`` is computed at call time so it reflects the true age of the last
    hardware reading, not the age of this snapshot object.
    """

    pan_deg: float
    sample_time_monotonic: float
    seq: int
    valid: bool
    status: TelemetryStatus

    @property
    def age_s(self) -> float:
        """Seconds elapsed since the hardware sample was received."""
        return time.monotonic() - self.sample_time_monotonic


_INITIALISING_SNAPSHOT = PanTelemetrySnapshot(
    pan_deg=0.0,
    sample_time_monotonic=0.0,
    seq=0,
    valid=False,
    status="initialising",
)


def _classify(
    snap: PanTelemetrySnapshot,
    stale_threshold_s: float,
    expired_threshold_s: float,
) -> PanTelemetrySnapshot:
    """Return *snap* with ``status`` and ``valid`` re-derived from current age.

    Parameters
    ----------
    snap : PanTelemetrySnapshot
        The stored snapshot to re-classify.
    stale_threshold_s : float
        Age in seconds above which a snapshot is classified as stale.
    expired_threshold_s : float
        Age in seconds above which a snapshot is classified as expired.

    Returns
    -------
    PanTelemetrySnapshot
        The original snapshot if still fresh or initialising; a new frozen
        snapshot with updated ``status`` and ``valid=False`` otherwise.
    """
    if snap.status == "initialising":
        return snap

    age = snap.age_s
    if age >= expired_threshold_s:
        new_status: TelemetryStatus = "expired"
    elif age >= stale_threshold_s:
        new_status = "stale"
    else:
        new_status = "fresh"

    if new_status == snap.status:
        return snap

    return PanTelemetrySnapshot(
        pan_deg=snap.pan_deg,
        sample_time_monotonic=snap.sample_time_monotonic,
        seq=snap.seq,
        valid=new_status == "fresh",
        status=new_status,
    )


class PanTelemetryPoller:
    """Background poller that owns the pan servo telemetry query path.

    Internally uses two daemon threads:

    - **Cadence thread** (``_run_loop``): sleeps for ``poll_interval_*_s``
      and signals the serial worker via a :class:`threading.Event`.  Its sleep
      is purely temporal — independent of serial timing.
    - **Serial worker** (``_serial_worker_loop``): waits for the cadence
      trigger then calls :meth:`~ugv_follower.control.ugv_controller.UGVController.query_pan_deg`
      with the configured ``query_timeout_s`` and publishes the result as an
      immutable :class:`PanTelemetrySnapshot`.

    Consumers call :meth:`get_snapshot` once per control cycle to obtain the
    latest reading without ever waiting on serial I/O.

    On timeout or serial error the old snapshot is left in place so that
    ``sample_time_monotonic`` naturally ages — causing the snapshot's status to
    degrade from ``"fresh"`` → ``"stale"`` → ``"expired"`` at the configured
    thresholds without any extra logic.

    Parameters
    ----------
    ugv_controller : UGVController
        Controller instance whose ``query_pan_deg()`` method is the sole serial
        query path (single arbiter contract).
    stale_threshold_s : float
        Seconds after the last valid reading before the snapshot is classified
        as stale.
    expired_threshold_s : float
        Seconds after the last valid reading before the snapshot is classified
        as expired (control should hold position).
    poll_interval_tracking_s : float
        Target interval between polls in tracking mode (fast cadence).
    poll_interval_idle_s : float
        Target interval between polls in idle/manual mode (slow cadence).
    query_timeout_s : float
        Serial read timeout passed to ``query_pan_deg()`` on every poll.
    """

    def __init__(
        self,
        ugv_controller: UGVController,
        stale_threshold_s: float,
        expired_threshold_s: float,
        poll_interval_tracking_s: float,
        poll_interval_idle_s: float,
        query_timeout_s: float,
    ) -> None:
        self._ugv = ugv_controller
        self._stale_threshold_s = stale_threshold_s
        self._expired_threshold_s = expired_threshold_s
        self._poll_interval_tracking_s = poll_interval_tracking_s
        self._poll_interval_idle_s = poll_interval_idle_s
        self._query_timeout_s = query_timeout_s

        # Shared state — protected by _lock
        self._snapshot: PanTelemetrySnapshot = _INITIALISING_SNAPSHOT
        self._seq: int = 0
        self._lock = threading.Lock()

        # Mode flag — protected by _mode_lock (separate to avoid blocking I/O)
        self._tracking_mode: bool = False
        self._mode_lock = threading.Lock()

        # Thread lifecycle
        self._stop_event = threading.Event()
        self._trigger_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._serial_thread: threading.Thread | None = None

        logger.debug(
            "PanTelemetryPoller initialised "
            "(stale={:.3f}s, expired={:.3f}s, "
            "tracking_interval={:.3f}s, idle_interval={:.3f}s, "
            "query_timeout={:.3f}s).",
            stale_threshold_s,
            expired_threshold_s,
            poll_interval_tracking_s,
            poll_interval_idle_s,
            query_timeout_s,
        )

    def start(self) -> None:
        """Start the background cadence and serial worker threads.

        Raises
        ------
        RuntimeError
            If the poller is already running.
        """
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("PanTelemetryPoller is already running.")
        self._stop_event.clear()
        self._trigger_event.clear()
        self._thread = threading.Thread(
            target=self._run_loop,
            name="PanTelemetryPollThread",
            daemon=True,
        )
        self._serial_thread = threading.Thread(
            target=self._serial_worker_loop,
            name="PanTelemetrySerialWorker",
            daemon=True,
        )
        self._thread.start()
        self._serial_thread.start()
        logger.info("PanTelemetryPoller started.")

    def stop(self) -> None:
        """Signal both threads to exit and wait for them to finish."""
        self._stop_event.set()
        self._trigger_event.set()  # wake serial worker if blocked on wait()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                logger.warning(
                    "PanTelemetryPoller cadence thread did not exit within 2 s timeout."
                )
            self._thread = None
        if self._serial_thread is not None:
            self._serial_thread.join(timeout=2.0)
            if self._serial_thread.is_alive():
                logger.warning(
                    "PanTelemetryPoller serial worker did not exit within 2 s timeout."
                )
            self._serial_thread = None
        logger.info("PanTelemetryPoller stopped.")

    def set_tracking_mode(self, tracking: bool) -> None:
        """Switch between fast (tracking) and slow (idle) poll cadences.

        Parameters
        ----------
        tracking : bool
            ``True`` for autonomous/tracking mode; ``False`` for manual/idle/estop.
        """
        with self._mode_lock:
            self._tracking_mode = tracking
        logger.debug(
            "PanTelemetryPoller: mode set to {}.",
            "tracking" if tracking else "idle",
        )

    def get_snapshot(self) -> PanTelemetrySnapshot:
        """Return the latest snapshot with freshness status derived from current age.

        The snapshot reference is copied under the lock then released before
        re-classification, so the lock is never held during the age computation.

        Returns
        -------
        PanTelemetrySnapshot
            The most recent coherent telemetry sample, re-classified to the
            correct ``status`` based on how long ago it was received.
        """
        with self._lock:
            snap = self._snapshot
        return _classify(snap, self._stale_threshold_s, self._expired_threshold_s)

    def _run_loop(self) -> None:
        """Cadence thread — fires the serial trigger at the configured interval.

        Sleeps for exactly ``poll_interval_*_s`` on each iteration.  Serial
        timing does not affect this sleep.
        """
        logger.debug("PanTelemetryPoller cadence thread running.")
        while not self._stop_event.is_set():
            with self._mode_lock:
                tracking = self._tracking_mode
            interval = (
                self._poll_interval_tracking_s
                if tracking
                else self._poll_interval_idle_s
            )
            self._trigger_event.set()
            self._stop_event.wait(interval)
        logger.debug("PanTelemetryPoller cadence thread exiting.")

    def _serial_worker_loop(self) -> None:
        """Serial worker thread — performs blocking pan angle queries.

        Waits for the cadence trigger, then calls ``query_pan_deg()`` with
        the configured ``query_timeout_s`` and publishes a fresh snapshot.
        On timeout the snapshot is left unchanged so it ages naturally.
        """
        logger.debug("PanTelemetryPoller serial worker running.")
        while not self._stop_event.is_set():
            triggered = self._trigger_event.wait(timeout=1.0)
            if not triggered or self._stop_event.is_set():
                continue
            self._trigger_event.clear()

            pan_deg = self._ugv.query_pan_deg(timeout_s=self._query_timeout_s)

            if pan_deg is not None:
                with self._lock:
                    next_seq = self._snapshot.seq + 1
                snap = PanTelemetrySnapshot(
                    pan_deg=pan_deg,
                    sample_time_monotonic=time.monotonic(),
                    seq=next_seq,
                    valid=True,
                    status="fresh",
                )
                with self._lock:
                    if snap.seq > self._snapshot.seq:
                        self._snapshot = snap
                logger.debug(
                    "Pan telemetry: fresh {:.2f}° seq={}.", pan_deg, snap.seq
                )
            else:
                logger.debug("Pan telemetry: query timed out — snapshot unchanged.")
        logger.debug("PanTelemetryPoller serial worker exiting.")
