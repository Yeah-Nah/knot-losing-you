"""Unit tests for the telemetry-only pan query diagnostic tool.

Module under test
-----------------
tools.check_pan_telemetry_only

Running
-------
From ``ugv-follower/``::

    pytest tests/test_check_pan_telemetry_only.py -v
"""

from __future__ import annotations

import sys
from collections.abc import Callable
from unittest.mock import MagicMock, patch

import pytest

from tools.check_pan_telemetry_only import (
    QueryResult,
    _compute_summary,
    _open_serial,
    _run_measurement_loop,
    _send_and_await_pan,
    main,
    run,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _readline_sequence(lines: list[bytes]) -> Callable[[], bytes]:
    """Return a callable that yields *lines* in order, then empty bytes."""
    remaining = list(lines)

    def _readline() -> bytes:
        if remaining:
            return remaining.pop(0)
        return b""

    return _readline


def _make_result(
    query_id: int,
    send_time: float,
    success: bool,
    latency_ms: float | None = None,
) -> QueryResult:
    return QueryResult(
        query_id=query_id,
        send_time_monotonic=send_time,
        success=success,
        latency_ms=latency_ms,
        pan_deg=1.0 if success else None,
        lines_read=1 if success else 0,
        non_telemetry_lines=0,
    )


class _FakeClock:
    """Shared monotonic clock for deterministic loop-timing tests."""

    def __init__(self) -> None:
        self.t = 0.0
        self.sleep_calls: list[float] = []

    def monotonic(self) -> float:
        return self.t

    def sleep(self, duration: float) -> None:
        self.sleep_calls.append(duration)
        self.t += duration

    def advance(self, duration: float) -> None:
        self.t += duration


# ---------------------------------------------------------------------------
# _open_serial
# ---------------------------------------------------------------------------


@patch("tools.check_pan_telemetry_only.serial.Serial")
@patch("tools.check_pan_telemetry_only.ensure_character_device_available")
def test_open_serial_runs_preflight_before_open(
    mock_preflight: MagicMock,
    mock_serial_cls: MagicMock,
) -> None:
    """_open_serial checks port ownership before opening the UART."""
    mock_serial_cls.return_value = MagicMock()

    result = _open_serial("/dev/ttyAMA0")

    mock_preflight.assert_called_once_with("/dev/ttyAMA0", device_label="Serial port")
    mock_serial_cls.assert_called_once_with("/dev/ttyAMA0", 115200, timeout=0.05)
    assert result is mock_serial_cls.return_value


@patch("tools.check_pan_telemetry_only.logger")
@patch("tools.check_pan_telemetry_only.serial.Serial")
@patch("tools.check_pan_telemetry_only.ensure_character_device_available")
def test_open_serial_returns_none_when_preflight_fails(
    mock_preflight: MagicMock,
    mock_serial_cls: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """_open_serial aborts cleanly when another process owns the serial port."""
    mock_preflight.side_effect = RuntimeError("Serial port /dev/ttyAMA0 is busy")

    result = _open_serial("/dev/ttyAMA0")

    assert result is None
    mock_serial_cls.assert_not_called()
    mock_logger.error.assert_called_once()


@patch("tools.check_pan_telemetry_only.logger")
@patch("tools.check_pan_telemetry_only.serial.Serial")
@patch("tools.check_pan_telemetry_only.ensure_character_device_available")
def test_open_serial_returns_none_when_open_fails(
    mock_preflight: MagicMock,
    mock_serial_cls: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """_open_serial returns None when serial.Serial raises SerialException."""
    import serial

    mock_serial_cls.side_effect = serial.SerialException("port vanished")

    result = _open_serial("/dev/ttyAMA0")

    assert result is None
    mock_logger.error.assert_called_once()


@patch("tools.check_pan_telemetry_only._run_measurement_loop")
@patch("tools.check_pan_telemetry_only._open_serial")
def test_run_exits_early_when_open_serial_returns_none(
    mock_open_serial: MagicMock,
    mock_loop: MagicMock,
) -> None:
    """run() does not attempt measurement when the port cannot be opened."""
    mock_open_serial.return_value = None

    run("/dev/ttyAMA0", 1.0, 0.05, 0.3, init_module=True, verbose_lines=False)

    mock_loop.assert_not_called()


# ---------------------------------------------------------------------------
# _compute_summary
# ---------------------------------------------------------------------------


def test_compute_summary_mixed_successes_and_timeouts() -> None:
    results = [
        _make_result(0, 0.0, True, latency_ms=10.0),
        _make_result(1, 0.1, False),
        _make_result(2, 0.2, True, latency_ms=20.0),
        _make_result(3, 0.3, True, latency_ms=30.0),
        _make_result(4, 0.4, False),
    ]

    stats = _compute_summary(results)

    assert stats.total_queries == 5
    assert stats.successful == 3
    assert stats.timeout_count == 2
    assert stats.success_ratio == pytest.approx(3 / 5)
    assert stats.latency_min_ms == pytest.approx(10.0)
    assert stats.latency_median_ms == pytest.approx(20.0)
    assert stats.latency_max_ms == pytest.approx(30.0)
    assert stats.latency_p95_ms is not None
    # Successful queries were sent at t=0.0, 0.2, 0.3 -> largest gap is 0.2 - 0.0.
    assert stats.longest_gap_s == pytest.approx(0.2)


def test_compute_summary_all_timeouts() -> None:
    results = [_make_result(i, float(i), False) for i in range(3)]

    stats = _compute_summary(results)

    assert stats.total_queries == 3
    assert stats.successful == 0
    assert stats.timeout_count == 3
    assert stats.success_ratio == 0.0
    assert stats.latency_min_ms is None
    assert stats.latency_median_ms is None
    assert stats.latency_p95_ms is None
    assert stats.latency_max_ms is None
    assert stats.longest_gap_s is None


def test_compute_summary_single_success() -> None:
    results = [_make_result(0, 0.0, True, latency_ms=15.0)]

    stats = _compute_summary(results)

    assert stats.successful == 1
    assert stats.latency_min_ms == pytest.approx(15.0)
    assert stats.latency_median_ms == pytest.approx(15.0)
    assert stats.latency_p95_ms == pytest.approx(15.0)
    assert stats.latency_max_ms == pytest.approx(15.0)
    assert stats.longest_gap_s is None


def test_compute_summary_empty_results() -> None:
    stats = _compute_summary([])

    assert stats.total_queries == 0
    assert stats.success_ratio == 0.0
    assert stats.longest_gap_s is None


# ---------------------------------------------------------------------------
# _send_and_await_pan
# ---------------------------------------------------------------------------


def test_send_and_await_pan_success() -> None:
    ser = MagicMock()
    ser.readline.side_effect = _readline_sequence([b'{"T":1001,"pan":12.5}\n'])

    result = _send_and_await_pan(ser, query_id=0, timeout_s=0.3, verbose_lines=False)

    assert result.success is True
    assert result.pan_deg == pytest.approx(12.5)
    assert result.lines_read == 1
    assert result.non_telemetry_lines == 0
    assert result.latency_ms is not None


def test_send_and_await_pan_timeout() -> None:
    ser = MagicMock()
    ser.readline.side_effect = _readline_sequence([])  # always empty

    result = _send_and_await_pan(ser, query_id=0, timeout_s=0.01, verbose_lines=False)

    assert result.success is False
    assert result.latency_ms is None
    assert result.pan_deg is None
    assert result.lines_read == 0


def test_send_and_await_pan_counts_non_telemetry_lines() -> None:
    ser = MagicMock()
    ser.readline.side_effect = _readline_sequence(
        [b'{"T":1005,"id":1,"status":0}\n', b'{"T":1001,"pan":5.0}\n']
    )

    result = _send_and_await_pan(ser, query_id=0, timeout_s=0.3, verbose_lines=False)

    assert result.success is True
    assert result.pan_deg == pytest.approx(5.0)
    assert result.lines_read == 2
    assert result.non_telemetry_lines == 1


def test_send_and_await_pan_tolerates_malformed_json() -> None:
    ser = MagicMock()
    ser.readline.side_effect = _readline_sequence(
        [b"not json\n", b'{"T":1001,"pan":3.0}\n']
    )

    result = _send_and_await_pan(ser, query_id=0, timeout_s=0.3, verbose_lines=False)

    assert result.success is True
    assert result.pan_deg == pytest.approx(3.0)
    assert result.lines_read == 2
    assert result.non_telemetry_lines == 1


def test_send_and_await_pan_flushes_before_write() -> None:
    ser = MagicMock()
    ser.readline.side_effect = _readline_sequence([b'{"T":1001,"pan":1.0}\n'])

    _send_and_await_pan(ser, query_id=0, timeout_s=0.3, verbose_lines=False)

    call_names = [call[0] for call in ser.mock_calls if call[0] in ("reset_input_buffer", "write")]
    assert call_names.index("reset_input_buffer") < call_names.index("write")


def test_send_and_await_pan_flush_disabled_skips_reset() -> None:
    ser = MagicMock()
    ser.readline.side_effect = _readline_sequence([b'{"T":1001,"pan":1.0}\n'])

    _send_and_await_pan(
        ser, query_id=0, timeout_s=0.3, verbose_lines=False, flush_before_query=False
    )

    ser.reset_input_buffer.assert_not_called()
    ser.write.assert_called_once()


@patch("tools.check_pan_telemetry_only.logger")
def test_send_and_await_pan_verbose_logs_non_pan_lines(mock_logger: MagicMock) -> None:
    ser = MagicMock()
    ser.readline.side_effect = _readline_sequence(
        [b'{"T":1005,"id":1,"status":0}\n', b'{"T":1001,"pan":5.0}\n']
    )

    _send_and_await_pan(ser, query_id=0, timeout_s=0.3, verbose_lines=True)

    mock_logger.debug.assert_called_once()


@patch("tools.check_pan_telemetry_only.logger")
def test_send_and_await_pan_quiet_without_verbose(mock_logger: MagicMock) -> None:
    ser = MagicMock()
    ser.readline.side_effect = _readline_sequence(
        [b'{"T":1005,"id":1,"status":0}\n', b'{"T":1001,"pan":5.0}\n']
    )

    _send_and_await_pan(ser, query_id=0, timeout_s=0.3, verbose_lines=False)

    mock_logger.debug.assert_not_called()


# ---------------------------------------------------------------------------
# _run_measurement_loop — fixed-slot cadence, fake clock
# ---------------------------------------------------------------------------


def test_run_measurement_loop_sleeps_full_interval_when_queries_are_fast(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Fast queries: cadence is genuinely poll_interval_s between sends."""
    import tools.check_pan_telemetry_only as module

    clock = _FakeClock()
    monkeypatch.setattr(module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(module.time, "sleep", clock.sleep)

    ser = MagicMock()

    def _fast_readline() -> bytes:
        return b'{"T":1001,"pan":1.0}\n'

    ser.readline.side_effect = _fast_readline

    results = _run_measurement_loop(
        ser, duration_s=0.12, poll_interval_s=0.05, timeout_s=0.3, verbose_lines=False
    )

    assert len(results) == 3
    assert all(r.success for r in results)
    assert clock.sleep_calls == pytest.approx([0.05, 0.05, 0.05])


def test_run_measurement_loop_does_not_correct_drift_when_queries_time_out(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Timeout-length queries: next_slot falls behind and no sleep occurs."""
    import tools.check_pan_telemetry_only as module

    clock = _FakeClock()
    monkeypatch.setattr(module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(module.time, "sleep", clock.sleep)

    ser = MagicMock()

    def _slow_readline() -> bytes:
        # Simulate a read that consumes the whole per-query timeout.
        clock.advance(0.3)
        return b""

    ser.readline.side_effect = _slow_readline

    results = _run_measurement_loop(
        ser, duration_s=0.35, poll_interval_s=0.05, timeout_s=0.3, verbose_lines=False
    )

    assert len(results) == 2
    assert all(not r.success for r in results)
    assert clock.sleep_calls == []


# ---------------------------------------------------------------------------
# main() CLI defaults
# ---------------------------------------------------------------------------


@patch("tools.check_pan_telemetry_only.run")
def test_main_uses_documented_defaults(mock_run: MagicMock, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(sys, "argv", ["ugv-check-pan-telemetry-only"])

    main()

    mock_run.assert_called_once_with(
        "/dev/ttyAMA0",
        30.0,
        0.05,
        0.3,
        init_module=True,
        verbose_lines=False,
        flush_before_query=True,
    )


@patch("tools.check_pan_telemetry_only.run")
def test_main_no_flush_flag_disables_flush(
    mock_run: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(sys, "argv", ["ugv-check-pan-telemetry-only", "--no-flush"])

    main()

    assert mock_run.call_args.kwargs["flush_before_query"] is False
