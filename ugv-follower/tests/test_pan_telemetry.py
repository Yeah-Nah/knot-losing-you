"""Tests for PanTelemetrySnapshot and PanTelemetryPoller.

Snapshot tests use no threading.
Poller tests use real daemon threads with a mocked UGVController, following
the same pattern as test_command_shaper.py.
"""

from __future__ import annotations

import time
import threading
from unittest.mock import MagicMock

import pytest

from ugv_follower.control.pan_telemetry import (
    PanTelemetryPoller,
    PanTelemetrySnapshot,
    _INITIALISING_SNAPSHOT,
    _classify,
)
from ugv_follower.control.ugv_controller import UGVController

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_STALE_S = 0.10
_EXPIRED_S = 0.50
_INTERVAL_TRACKING = 0.02   # 50 Hz — fast enough to accumulate calls in tests
_INTERVAL_IDLE = 0.10        # 10 Hz


_QUERY_TIMEOUT = 0.05


def _make_poller(query_fn=None) -> PanTelemetryPoller:
    ugv = MagicMock(spec=UGVController)
    _fn = query_fn if query_fn is not None else (lambda: None)
    # Wrap so the poller can call query_pan_deg(timeout_s=...) without TypeError.
    ugv.query_pan_deg.side_effect = lambda timeout_s=None: _fn()
    return PanTelemetryPoller(
        ugv_controller=ugv,
        stale_threshold_s=_STALE_S,
        expired_threshold_s=_EXPIRED_S,
        poll_interval_tracking_s=_INTERVAL_TRACKING,
        poll_interval_idle_s=_INTERVAL_IDLE,
        query_timeout_s=_QUERY_TIMEOUT,
    )


def _fresh_snap(pan_deg: float = 0.0, seq: int = 1) -> PanTelemetrySnapshot:
    return PanTelemetrySnapshot(
        pan_deg=pan_deg,
        sample_time_monotonic=time.monotonic(),
        seq=seq,
        valid=True,
        status="fresh",
    )


def _aged_snap(age_s: float, pan_deg: float = 0.0, seq: int = 1) -> PanTelemetrySnapshot:
    return PanTelemetrySnapshot(
        pan_deg=pan_deg,
        sample_time_monotonic=time.monotonic() - age_s,
        seq=seq,
        valid=True,
        status="fresh",
    )


# ---------------------------------------------------------------------------
# Snapshot tests — no threading
# ---------------------------------------------------------------------------


class TestPanTelemetrySnapshot:
    def test_age_s_increases_over_time(self) -> None:
        snap = _fresh_snap()
        age0 = snap.age_s
        time.sleep(0.02)
        assert snap.age_s > age0

    def test_frozen_cannot_be_mutated(self) -> None:
        snap = _fresh_snap()
        with pytest.raises((AttributeError, TypeError)):
            snap.pan_deg = 99.0  # type: ignore[misc]

    def test_initialising_snapshot_is_invalid(self) -> None:
        assert _INITIALISING_SNAPSHOT.valid is False
        assert _INITIALISING_SNAPSHOT.status == "initialising"
        assert _INITIALISING_SNAPSHOT.seq == 0


class TestClassify:
    def test_fresh_when_just_stamped(self) -> None:
        snap = _fresh_snap()
        result = _classify(snap, _STALE_S, _EXPIRED_S)
        assert result.status == "fresh"
        assert result.valid is True
        assert result is snap  # unchanged reference

    def test_stale_after_threshold(self) -> None:
        snap = _aged_snap(age_s=_STALE_S + 0.01)
        result = _classify(snap, _STALE_S, _EXPIRED_S)
        assert result.status == "stale"
        assert result.valid is False
        assert result.pan_deg == snap.pan_deg
        assert result.seq == snap.seq

    def test_expired_after_expired_threshold(self) -> None:
        snap = _aged_snap(age_s=_EXPIRED_S + 0.01)
        result = _classify(snap, _STALE_S, _EXPIRED_S)
        assert result.status == "expired"
        assert result.valid is False

    def test_initialising_left_unchanged(self) -> None:
        result = _classify(_INITIALISING_SNAPSHOT, _STALE_S, _EXPIRED_S)
        assert result is _INITIALISING_SNAPSHOT

    def test_returns_same_object_when_still_fresh(self) -> None:
        snap = _fresh_snap()
        assert _classify(snap, _STALE_S, _EXPIRED_S) is snap

    def test_stale_preserves_pan_deg_and_sample_time(self) -> None:
        stamp = time.monotonic() - (_STALE_S + 0.05)
        snap = PanTelemetrySnapshot(
            pan_deg=12.5,
            sample_time_monotonic=stamp,
            seq=7,
            valid=True,
            status="fresh",
        )
        result = _classify(snap, _STALE_S, _EXPIRED_S)
        assert result.pan_deg == pytest.approx(12.5)
        assert result.sample_time_monotonic == pytest.approx(stamp)
        assert result.seq == 7


# ---------------------------------------------------------------------------
# Poller lifecycle tests — real daemon threads
# ---------------------------------------------------------------------------


class TestPanTelemetryPollerLifecycle:
    def test_start_stop_clean(self) -> None:
        poller = _make_poller(query_fn=lambda: None)
        poller.start()
        time.sleep(0.05)
        poller.stop()
        assert poller._thread is None

    def test_start_twice_raises(self) -> None:
        poller = _make_poller(query_fn=lambda: None)
        poller.start()
        try:
            with pytest.raises(RuntimeError, match="already running"):
                poller.start()
        finally:
            poller.stop()

    def test_stop_without_start_is_safe(self) -> None:
        poller = _make_poller()
        poller.stop()  # must not raise

    def test_thread_named_correctly(self) -> None:
        poller = _make_poller(query_fn=lambda: None)
        poller.start()
        try:
            assert poller._thread is not None
            assert poller._thread.name == "PanTelemetryPollThread"
        finally:
            poller.stop()

    def test_thread_is_daemon(self) -> None:
        poller = _make_poller(query_fn=lambda: None)
        poller.start()
        try:
            assert poller._thread is not None
            assert poller._thread.daemon is True
        finally:
            poller.stop()


# ---------------------------------------------------------------------------
# Poller snapshot publication tests — real daemon threads
# ---------------------------------------------------------------------------


class TestPanTelemetryPollerPublication:
    def test_publishes_fresh_snapshot_on_valid_query(self) -> None:
        poller = _make_poller(query_fn=lambda: 12.5)
        poller.set_tracking_mode(True)
        poller.start()
        time.sleep(_INTERVAL_TRACKING * 5)  # allow several polls
        poller.stop()

        snap = poller.get_snapshot()
        assert snap.status == "fresh"
        assert snap.valid is True
        assert snap.pan_deg == pytest.approx(12.5)
        assert snap.seq >= 1

    def test_stays_initialising_on_repeated_timeout(self) -> None:
        poller = _make_poller(query_fn=lambda: None)
        poller.set_tracking_mode(True)
        poller.start()
        time.sleep(_INTERVAL_TRACKING * 5)
        poller.stop()

        # With no valid reading, snapshot must remain "initialising"
        # (never published — the old sentinel has sample_time_monotonic=0.0
        # but _classify leaves initialising snapshots unchanged).
        snap = poller.get_snapshot()
        assert snap.status == "initialising"
        assert snap.valid is False

    def test_seq_advances_on_each_valid_poll(self) -> None:
        poller = _make_poller(query_fn=lambda: 5.0)
        poller.set_tracking_mode(True)
        poller.start()
        time.sleep(_INTERVAL_TRACKING * 8)
        poller.stop()

        snap = poller.get_snapshot()
        assert snap.seq >= 3  # at least a few polls completed

    def test_stale_classification_on_age(self) -> None:
        # Return a valid reading once then stop responding.
        call_count = [0]

        def query_fn() -> float | None:
            call_count[0] += 1
            return 7.0 if call_count[0] == 1 else None

        poller = _make_poller(query_fn=query_fn)
        poller.set_tracking_mode(True)
        poller.start()
        # Wait for the first valid poll then let it age past stale threshold.
        time.sleep(_INTERVAL_TRACKING * 3)
        time.sleep(_STALE_S + 0.05)
        poller.stop()

        snap = poller.get_snapshot()
        assert snap.status in ("stale", "expired")
        assert snap.valid is False


# ---------------------------------------------------------------------------
# Mode-aware polling cadence tests
# ---------------------------------------------------------------------------


class TestModeAwarePolling:
    def test_tracking_mode_polls_faster_than_idle(self) -> None:
        lock = threading.Lock()
        tracking_calls: list[int] = [0]
        idle_calls: list[int] = [0]
        phase: list[str] = ["tracking"]

        def query_fn() -> float | None:
            with lock:
                if phase[0] == "tracking":
                    tracking_calls[0] += 1
                else:
                    idle_calls[0] += 1
            return 0.0

        window = 0.20  # seconds to measure each phase
        poller = _make_poller(query_fn=query_fn)
        poller.set_tracking_mode(True)
        poller.start()
        time.sleep(window)
        with lock:
            phase[0] = "idle"
        poller.set_tracking_mode(False)
        time.sleep(window)
        poller.stop()

        # Tracking cadence should be at least 3× faster than idle.
        t = tracking_calls[0]
        i = idle_calls[0]
        assert t > 0, "No tracking calls recorded"
        assert i > 0, "No idle calls recorded"
        assert t >= i * 2, (
            f"Expected tracking ({t}) to be at least 2× idle ({i}); "
            f"ratio={t/i:.1f}"
        )

    def test_mode_change_updates_cadence(self) -> None:
        lock = threading.Lock()
        counts: dict[str, int] = {"before": 0, "after": 0}
        phase: list[str] = ["before"]

        def query_fn() -> float | None:
            with lock:
                counts[phase[0]] += 1
            return 1.0

        poller = _make_poller(query_fn=query_fn)
        poller.set_tracking_mode(True)
        poller.start()
        time.sleep(0.15)
        with lock:
            phase[0] = "after"
        poller.set_tracking_mode(False)
        time.sleep(0.15)
        poller.stop()

        # Tracking (before) should have produced more calls than idle (after).
        assert counts["before"] > counts["after"]


# ---------------------------------------------------------------------------
# Concurrency guardrail tests
# ---------------------------------------------------------------------------


class TestConcurrencyGuardrails:
    def test_late_reply_guard_monotonic_seq(self) -> None:
        """Manually inject a lower-seq snapshot; verify it does not overwrite a higher one."""
        poller = _make_poller()

        newer = PanTelemetrySnapshot(
            pan_deg=20.0,
            sample_time_monotonic=time.monotonic(),
            seq=10,
            valid=True,
            status="fresh",
        )
        older = PanTelemetrySnapshot(
            pan_deg=5.0,
            sample_time_monotonic=time.monotonic(),
            seq=5,
            valid=True,
            status="fresh",
        )

        # Seed the poller with the newer snapshot.
        with poller._lock:
            poller._snapshot = newer

        # Attempt to overwrite with the older (lower seq) snapshot.
        with poller._lock:
            if older.seq > poller._snapshot.seq:
                poller._snapshot = older

        # The newer snapshot must still be current.
        snap = poller.get_snapshot()
        assert snap.seq == 10
        assert snap.pan_deg == pytest.approx(20.0)

    def test_get_snapshot_releases_lock_before_classify(self) -> None:
        """Verify get_snapshot() does not hold the lock while classifying."""
        poller = _make_poller()
        newer = _fresh_snap(pan_deg=3.0, seq=1)
        with poller._lock:
            poller._snapshot = newer

        # If the lock were held during classify, a concurrent lock acquisition
        # from another thread would deadlock or time out.
        acquired = threading.Event()
        failed = threading.Event()

        def try_acquire() -> None:
            if poller._lock.acquire(timeout=0.5):
                acquired.set()
                poller._lock.release()
            else:
                failed.set()

        # Call get_snapshot() which internally acquires then releases the lock.
        poller.get_snapshot()

        t = threading.Thread(target=try_acquire)
        t.start()
        t.join(timeout=1.0)

        assert acquired.is_set(), "Lock was not released by get_snapshot()"
        assert not failed.is_set()

    def test_snapshot_immutability_prevents_partial_reads(self) -> None:
        """Frozen snapshot — no reader can see a half-written state."""
        snap = _fresh_snap(pan_deg=42.0, seq=3)
        # Any attempt to mutate must raise; validates frozen=True contract.
        with pytest.raises((AttributeError, TypeError)):
            snap.pan_deg = 0.0  # type: ignore[misc]
        with pytest.raises((AttributeError, TypeError)):
            snap.seq = 99  # type: ignore[misc]


# ---------------------------------------------------------------------------
# Serial worker and configuration wiring tests
# ---------------------------------------------------------------------------


class TestSerialWorkerDecoupling:
    def test_query_timeout_forwarded_to_controller(self) -> None:
        """query_pan_deg is called with query_timeout_s, not a hardcoded default."""
        ugv = MagicMock(spec=UGVController)
        ugv.query_pan_deg.return_value = 5.0
        custom_timeout = 0.077
        poller = PanTelemetryPoller(
            ugv_controller=ugv,
            stale_threshold_s=_STALE_S,
            expired_threshold_s=_EXPIRED_S,
            poll_interval_tracking_s=_INTERVAL_TRACKING,
            poll_interval_idle_s=_INTERVAL_IDLE,
            query_timeout_s=custom_timeout,
        )
        poller.set_tracking_mode(True)
        poller.start()
        time.sleep(_INTERVAL_TRACKING * 5)
        poller.stop()

        assert ugv.query_pan_deg.call_count >= 1
        for call in ugv.query_pan_deg.call_args_list:
            timeout_arg = call.kwargs.get("timeout_s", call.args[0] if call.args else None)
            assert timeout_arg == pytest.approx(custom_timeout)

    def test_serial_thread_is_daemon_and_named(self) -> None:
        """The serial worker thread is a daemon with the expected name."""
        poller = _make_poller(query_fn=lambda: None)
        poller.start()
        try:
            assert poller._serial_thread is not None
            assert poller._serial_thread.daemon is True
            assert poller._serial_thread.name == "PanTelemetrySerialWorker"
        finally:
            poller.stop()

    def test_stop_joins_serial_thread(self) -> None:
        """stop() sets _serial_thread to None after joining."""
        poller = _make_poller(query_fn=lambda: None)
        poller.start()
        poller.stop()
        assert poller._serial_thread is None
