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

import json
import pathlib
import sys
from collections.abc import Callable
from unittest.mock import MagicMock, call, patch

import pytest

from tools.check_pan_telemetry_only import (
    QueryResult,
    SummaryStats,
    _build_pre_commands,
    _compute_summary,
    _load_pre_command_jsonl,
    _log_query_result,
    _log_summary,
    _maybe_init,
    _matched_ratio,
    _median_inter_success_gap_s,
    _open_serial,
    _parse_command_payload,
    _run_measurement_loop,
    _send_and_await_pan,
    _unique_pan_count,
    _write_command,
    _write_jsonl_record,
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
        use_seq_token=False,
        token_field="S",
        linger_s=0.0,
        retain_raw_lines=False,
        log_jsonl_path=None,
        pre_cmd_json=None,
        pre_cmd_jsonl=None,
        pre_cmd_delay_s=0.1,
        tx_log_jsonl_path=None,
    )


@patch("tools.check_pan_telemetry_only.run")
def test_main_no_flush_flag_disables_flush(
    mock_run: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(sys, "argv", ["ugv-check-pan-telemetry-only", "--no-flush"])

    main()

    assert mock_run.call_args.kwargs["flush_before_query"] is False


@patch("tools.check_pan_telemetry_only.run")
def test_main_seq_token_flags_forwarded(
    mock_run: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        sys,
        "argv",
        ["ugv-check-pan-telemetry-only", "--seq-token", "--token-field", "seq"],
    )

    main()

    assert mock_run.call_args.kwargs["use_seq_token"] is True
    assert mock_run.call_args.kwargs["token_field"] == "seq"


@patch("tools.check_pan_telemetry_only.run")
def test_main_linger_ms_converted_to_seconds(
    mock_run: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        sys, "argv", ["ugv-check-pan-telemetry-only", "--linger-ms", "50"]
    )

    main()

    assert mock_run.call_args.kwargs["linger_s"] == pytest.approx(0.05)


@patch("tools.check_pan_telemetry_only.run")
def test_main_retain_raw_lines_and_log_jsonl_forwarded(
    mock_run: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "ugv-check-pan-telemetry-only",
            "--retain-raw-lines",
            "--log-jsonl",
            "run.jsonl",
        ],
    )

    main()

    assert mock_run.call_args.kwargs["retain_raw_lines"] is True
    assert mock_run.call_args.kwargs["log_jsonl_path"] == "run.jsonl"


# ---------------------------------------------------------------------------
# Phase 0 — default-mode output regression (comparability contract)
# ---------------------------------------------------------------------------


@patch("tools.check_pan_telemetry_only.logger")
def test_log_query_result_default_mode_output_unchanged(mock_logger: MagicMock) -> None:
    """New optional fields left at their defaults must not add log lines."""
    success = _make_result(0, 1.0, True, latency_ms=12.5)
    _log_query_result(success)
    mock_logger.success.assert_called_once_with(
        "query_id={} send_t={:.3f} outcome=success latency_ms={:.2f} "
        "pan_deg={:.4f} lines_read={} non_telemetry_lines={}",
        0,
        1.0,
        12.5,
        1.0,
        1,
        0,
    )
    mock_logger.debug.assert_not_called()

    mock_logger.reset_mock()
    timeout = _make_result(1, 2.0, False)
    _log_query_result(timeout)
    mock_logger.warning.assert_called_once_with(
        "query_id={} send_t={:.3f} outcome=timeout latency_ms=n/a "
        "pan_deg=n/a lines_read={} non_telemetry_lines={}",
        1,
        2.0,
        0,
        0,
    )
    mock_logger.debug.assert_not_called()


@patch("tools.check_pan_telemetry_only.logger")
def test_log_summary_default_mode_suppresses_extended_lines(
    mock_logger: MagicMock,
) -> None:
    """Extended metrics must stay hidden unless show_extended=True is passed."""
    stats = SummaryStats(
        total_queries=2,
        successful=2,
        timeout_count=0,
        success_ratio=1.0,
        latency_min_ms=1.0,
        latency_median_ms=1.0,
        latency_p95_ms=1.0,
        latency_max_ms=1.0,
        longest_gap_s=0.5,
        matched_ratio=1.0,
        median_inter_success_gap_s=0.5,
        unique_pan_count=1,
        late_reply_after_timeout_count=3,
    )

    _log_summary(stats)

    logged_text = " ".join(str(call.args[0]) for call in mock_logger.info.call_args_list)
    assert "rho" not in logged_text
    assert "g~" not in logged_text
    assert "C_uniq" not in logged_text
    assert "linger" not in logged_text


@patch("tools.check_pan_telemetry_only.logger")
def test_log_summary_extended_mode_shows_new_lines(mock_logger: MagicMock) -> None:
    stats = SummaryStats(
        total_queries=2,
        successful=2,
        timeout_count=0,
        success_ratio=1.0,
        latency_min_ms=1.0,
        latency_median_ms=1.0,
        latency_p95_ms=1.0,
        latency_max_ms=1.0,
        longest_gap_s=0.5,
        matched_ratio=0.75,
        median_inter_success_gap_s=0.42,
        unique_pan_count=3,
        late_reply_after_timeout_count=2,
    )

    _log_summary(stats, show_extended=True)

    logged_text = " ".join(str(call.args[0]) for call in mock_logger.info.call_args_list)
    assert "rho" in logged_text
    assert "g~" in logged_text
    assert "C_uniq" in logged_text
    assert "linger" in logged_text


# ---------------------------------------------------------------------------
# _send_and_await_pan — edge timestamps, linger, token correlation
# ---------------------------------------------------------------------------


def test_send_and_await_pan_edge_timestamps_are_ordered() -> None:
    ser = MagicMock()
    ser.readline.side_effect = _readline_sequence([b'{"T":1001,"pan":12.5}\n'])

    result = _send_and_await_pan(ser, query_id=0, timeout_s=0.3, verbose_lines=False)

    assert result.first_byte_ts is not None
    assert result.parse_ts is not None
    assert result.timeout_ts is not None
    assert result.send_time_monotonic <= result.first_byte_ts <= result.parse_ts


def test_send_and_await_pan_timeout_records_timeout_ts_but_no_reply_ts() -> None:
    ser = MagicMock()
    ser.readline.side_effect = _readline_sequence([])

    result = _send_and_await_pan(ser, query_id=0, timeout_s=0.01, verbose_lines=False)

    assert result.timeout_ts is not None
    assert result.first_byte_ts is None
    assert result.parse_ts is None


def test_send_and_await_pan_linger_captures_late_reply_without_flipping_success(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A reply that lands only after the nominal deadline, but within the
    linger window, must be flagged without ever setting success=True."""
    import tools.check_pan_telemetry_only as module

    clock = _FakeClock()
    monkeypatch.setattr(module.time, "monotonic", clock.monotonic)

    ser = MagicMock()
    call_count = {"n": 0}

    def _readline() -> bytes:
        call_count["n"] += 1
        if call_count["n"] == 1:
            # First read during the nominal window: advance past the deadline.
            clock.advance(0.02)
            return b""
        # Second read, now inside the linger window: the late reply.
        return b'{"T":1001,"pan":9.0}\n'

    ser.readline.side_effect = _readline

    result = _send_and_await_pan(
        ser, query_id=0, timeout_s=0.01, verbose_lines=False, linger_s=0.2
    )

    assert result.success is False
    assert result.pan_deg is None
    assert result.linger_late_reply is True


def test_send_and_await_pan_linger_disabled_by_default() -> None:
    ser = MagicMock()
    ser.readline.side_effect = _readline_sequence([])

    result = _send_and_await_pan(ser, query_id=0, timeout_s=0.01, verbose_lines=False)

    assert result.linger_late_reply is False


def test_send_and_await_pan_token_match_success() -> None:
    ser = MagicMock()
    ser.readline.side_effect = _readline_sequence(
        [b'{"T":1001,"pan":1.0,"S":7}\n']
    )

    result = _send_and_await_pan(
        ser, query_id=0, timeout_s=0.3, verbose_lines=False, seq_token=7
    )

    assert result.seq_token == 7
    assert result.echoed_token == 7
    assert result.token_matched is True


def test_send_and_await_pan_token_mismatch() -> None:
    ser = MagicMock()
    ser.readline.side_effect = _readline_sequence(
        [b'{"T":1001,"pan":1.0,"S":99}\n']
    )

    result = _send_and_await_pan(
        ser, query_id=0, timeout_s=0.3, verbose_lines=False, seq_token=7
    )

    assert result.token_matched is False


def test_send_and_await_pan_token_field_absent_degrades_gracefully() -> None:
    ser = MagicMock()
    ser.readline.side_effect = _readline_sequence([b'{"T":1001,"pan":1.0}\n'])

    result = _send_and_await_pan(
        ser, query_id=0, timeout_s=0.3, verbose_lines=False, seq_token=7
    )

    assert result.seq_token == 7
    assert result.echoed_token is None
    assert result.token_matched is None


def test_send_and_await_pan_no_token_use_leaves_fields_none() -> None:
    ser = MagicMock()
    ser.readline.side_effect = _readline_sequence([b'{"T":1001,"pan":1.0}\n'])

    result = _send_and_await_pan(ser, query_id=0, timeout_s=0.3, verbose_lines=False)

    assert result.seq_token is None
    assert result.echoed_token is None
    assert result.token_matched is None


def test_send_and_await_pan_retain_raw_lines_populates_tuple() -> None:
    ser = MagicMock()
    ser.readline.side_effect = _readline_sequence(
        [b"not json\n", b'{"T":1005,"id":1}\n', b'{"T":1001,"pan":1.0}\n']
    )

    result = _send_and_await_pan(
        ser, query_id=0, timeout_s=0.3, verbose_lines=False, retain_raw_lines=True
    )

    assert result.raw_lines is not None
    assert len(result.raw_lines) == 3
    assert result.raw_lines[0][2] is False  # malformed JSON: parsed_ok=False
    assert result.raw_lines[1][2] is True  # valid JSON, non-pan: parsed_ok=True
    assert result.raw_lines[2][2] is True  # pan line: parsed_ok=True


def test_send_and_await_pan_raw_lines_absent_by_default() -> None:
    ser = MagicMock()
    ser.readline.side_effect = _readline_sequence([b'{"T":1001,"pan":1.0}\n'])

    result = _send_and_await_pan(ser, query_id=0, timeout_s=0.3, verbose_lines=False)

    assert result.raw_lines is None


# ---------------------------------------------------------------------------
# _write_jsonl_record
# ---------------------------------------------------------------------------


def test_write_jsonl_record_schema() -> None:
    result = QueryResult(
        query_id=0,
        send_time_monotonic=1.0,
        success=True,
        latency_ms=5.0,
        pan_deg=10.0,
        lines_read=1,
        non_telemetry_lines=0,
        seq_token=3,
        echoed_token=3,
        token_matched=True,
        first_byte_ts=1.001,
        parse_ts=1.002,
        timeout_ts=1.3,
        linger_late_reply=False,
        raw_lines=((1.001, '{"T":1001,"pan":10.0}', True),),
    )
    fake_file = MagicMock()

    _write_jsonl_record(fake_file, result)

    written = fake_file.write.call_args.args[0]
    record = json.loads(written)
    assert record["query_id"] == 0
    assert record["seq_token"] == 3
    assert record["token_matched"] is True
    assert record["raw_lines"] == [[1.001, '{"T":1001,"pan":10.0}', True]]
    assert written.endswith("\n")


# ---------------------------------------------------------------------------
# New summary aggregates — matched_ratio, median gap, unique pan count
# ---------------------------------------------------------------------------


def _token_result(
    query_id: int, send_time: float, token_matched: bool | None, pan_deg: float = 1.0
) -> QueryResult:
    return QueryResult(
        query_id=query_id,
        send_time_monotonic=send_time,
        success=True,
        latency_ms=5.0,
        pan_deg=pan_deg,
        lines_read=1,
        non_telemetry_lines=0,
        seq_token=query_id,
        echoed_token=query_id if token_matched else None,
        token_matched=token_matched,
    )


def test_matched_ratio_none_when_token_not_used() -> None:
    results = [_make_result(0, 0.0, True, latency_ms=1.0)]
    assert _matched_ratio(results, results) is None


def test_matched_ratio_computed_from_successes_only() -> None:
    results = [
        _token_result(0, 0.0, True),
        _token_result(1, 0.1, False),
        _token_result(2, 0.2, True),
    ]
    successes = results
    assert _matched_ratio(results, successes) == pytest.approx(2 / 3)


def test_median_inter_success_gap_distinct_from_longest_gap() -> None:
    # Gaps are [0.1, 1.0]; median of the two (0.55) differs from the longest
    # gap (1.0), which is exactly the distinction this metric exists for.
    successes = [
        _make_result(0, 0.0, True, latency_ms=1.0),
        _make_result(1, 0.1, True, latency_ms=1.0),
        _make_result(2, 1.1, True, latency_ms=1.0),
    ]
    assert _median_inter_success_gap_s(successes) == pytest.approx(0.55)


def test_unique_pan_count_counts_distinct_values() -> None:
    successes = [
        _token_result(0, 0.0, None, pan_deg=1.0),
        _token_result(1, 0.1, None, pan_deg=1.0),
        _token_result(2, 0.2, None, pan_deg=2.0),
    ]
    assert _unique_pan_count(successes) == 2


def test_compute_summary_includes_late_reply_count() -> None:
    late = QueryResult(
        query_id=0,
        send_time_monotonic=0.0,
        success=False,
        latency_ms=None,
        pan_deg=None,
        lines_read=1,
        non_telemetry_lines=0,
        linger_late_reply=True,
    )
    stats = _compute_summary([late])
    assert stats.late_reply_after_timeout_count == 1


# ---------------------------------------------------------------------------
# _parse_command_payload / _load_pre_command_jsonl / _build_pre_commands
# ---------------------------------------------------------------------------


def test_parse_command_payload_rejects_invalid_json() -> None:
    with pytest.raises(ValueError, match="--pre-cmd-json"):
        _parse_command_payload("not valid json{", source="--pre-cmd-json")


def test_parse_command_payload_rejects_missing_t() -> None:
    with pytest.raises(ValueError, match="--pre-cmd-json"):
        _parse_command_payload('{"cmd":0}', source="--pre-cmd-json")


def test_parse_command_payload_rejects_non_integer_t() -> None:
    with pytest.raises(ValueError, match="--pre-cmd-json"):
        _parse_command_payload('{"T":"130"}', source="--pre-cmd-json")


def test_parse_command_payload_rejects_bool_t() -> None:
    """`bool` is an `int` subclass in Python, so it needs an explicit reject."""
    with pytest.raises(ValueError, match="--pre-cmd-json"):
        _parse_command_payload('{"T":true}', source="--pre-cmd-json")


def test_parse_command_payload_accepts_valid_command() -> None:
    result = _parse_command_payload('{"T":123,"cmd":0}', source="--pre-cmd-json")
    assert result == {"T": 123, "cmd": 0}


def test_load_pre_command_jsonl_invalid_line_reports_line_number(
    tmp_path: pathlib.Path,
) -> None:
    path = tmp_path / "cmds.jsonl"
    path.write_text('{"T":1}\nnot valid json{\n')

    with pytest.raises(ValueError, match=r"--pre-cmd-jsonl.*line 2"):
        _load_pre_command_jsonl(str(path))


def test_build_pre_commands_empty_when_no_sources() -> None:
    assert _build_pre_commands(None, None) == []


def test_build_pre_commands_single_json_only() -> None:
    commands = _build_pre_commands('{"T":123,"cmd":0}', None)
    assert commands == [{"T": 123, "cmd": 0}]


def test_build_pre_commands_jsonl_only_in_file_order(tmp_path: pathlib.Path) -> None:
    path = tmp_path / "cmds.jsonl"
    path.write_text('{"T":1}\n{"T":2}\n{"T":3}\n')

    commands = _build_pre_commands(None, str(path))

    assert commands == [{"T": 1}, {"T": 2}, {"T": 3}]


def test_build_pre_commands_json_first_then_jsonl(tmp_path: pathlib.Path) -> None:
    path = tmp_path / "cmds.jsonl"
    path.write_text('{"T":2}\n{"T":3}\n')

    commands = _build_pre_commands('{"T":1}', str(path))

    assert commands == [{"T": 1}, {"T": 2}, {"T": 3}]


def test_build_pre_commands_skips_blank_jsonl_lines(tmp_path: pathlib.Path) -> None:
    path = tmp_path / "cmds.jsonl"
    path.write_text('{"T":1}\n\n   \n{"T":2}\n')

    commands = _build_pre_commands(None, str(path))

    assert commands == [{"T": 1}, {"T": 2}]


# ---------------------------------------------------------------------------
# _write_command
# ---------------------------------------------------------------------------


def test_write_command_writes_compact_json_and_newline() -> None:
    ser = MagicMock()

    _write_command(ser, {"T": 130}, phase="query")

    ser.write.assert_called_once_with(b'{"T":130}\n')


def test_write_command_appends_tx_log_record_when_enabled(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import tools.check_pan_telemetry_only as module

    monkeypatch.setattr(module.time, "monotonic", lambda: 42.0)
    ser = MagicMock()
    tx_log_file = MagicMock()

    _write_command(ser, {"T": 130}, phase="query", tx_log_file=tx_log_file)

    written = tx_log_file.write.call_args.args[0]
    record = json.loads(written)
    assert record["ts_monotonic"] == pytest.approx(42.0)
    assert record["phase"] == "query"
    assert record["payload"] == {"T": 130}
    assert written.endswith("\n")


def test_write_command_skips_tx_log_when_disabled() -> None:
    ser = MagicMock()

    _write_command(ser, {"T": 130}, phase="query", tx_log_file=None)

    ser.write.assert_called_once()


# ---------------------------------------------------------------------------
# Wire-format regression — locks in the _write_command refactor as a no-op
# ---------------------------------------------------------------------------


def test_send_and_await_pan_default_query_wire_format_unchanged() -> None:
    ser = MagicMock()
    ser.readline.side_effect = _readline_sequence([b'{"T":1001,"pan":1.0}\n'])

    _send_and_await_pan(ser, query_id=0, timeout_s=0.3, verbose_lines=False)

    ser.write.assert_called_once_with(b'{"T":130}\n')


@patch("tools.check_pan_telemetry_only._write_command")
def test_maybe_init_writes_via_write_command_with_init_phase(
    mock_write_command: MagicMock,
) -> None:
    ser = MagicMock()

    _maybe_init(ser, True)

    mock_write_command.assert_called_once_with(
        ser, {"T": 900, "main": 2, "module": 2}, phase="init", tx_log_file=None
    )


# ---------------------------------------------------------------------------
# run() — pre-command dispatch and TX logging integration
# ---------------------------------------------------------------------------


def _fake_serial_context() -> MagicMock:
    """A MagicMock serial connection that is its own context manager."""
    ser = MagicMock()
    ser.__enter__.return_value = ser
    return ser


@patch("tools.check_pan_telemetry_only._open_serial")
def test_run_sends_single_pre_command_before_first_query(
    mock_open_serial: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    import tools.check_pan_telemetry_only as module

    clock = _FakeClock()
    monkeypatch.setattr(module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(module.time, "sleep", clock.sleep)

    ser = _fake_serial_context()
    ser.readline.side_effect = lambda: b'{"T":1001,"pan":1.0}\n'
    mock_open_serial.return_value = ser

    run(
        "/dev/ttyAMA0",
        0.01,
        0.05,
        0.3,
        init_module=False,
        verbose_lines=False,
        pre_cmd_json='{"T":123,"cmd":0}',
    )

    assert ser.write.call_args_list[0] == call(b'{"T":123,"cmd":0}\n')
    assert ser.write.call_args_list[1] == call(b'{"T":130}\n')


@patch("tools.check_pan_telemetry_only._open_serial")
def test_run_sends_jsonl_pre_commands_in_file_order(
    mock_open_serial: MagicMock,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: pathlib.Path,
) -> None:
    import tools.check_pan_telemetry_only as module

    clock = _FakeClock()
    monkeypatch.setattr(module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(module.time, "sleep", clock.sleep)

    path = tmp_path / "cmds.jsonl"
    path.write_text('{"T":1}\n{"T":2}\n{"T":3}\n')

    ser = _fake_serial_context()
    mock_open_serial.return_value = ser

    run(
        "/dev/ttyAMA0",
        0.0,
        0.05,
        0.3,
        init_module=False,
        verbose_lines=False,
        pre_cmd_jsonl=str(path),
        pre_cmd_delay_s=0.02,
    )

    assert ser.write.call_args_list == [
        call(b'{"T":1}\n'),
        call(b'{"T":2}\n'),
        call(b'{"T":3}\n'),
    ]
    assert clock.sleep_calls[-3:] == pytest.approx([0.02, 0.02, 0.02])


@patch("tools.check_pan_telemetry_only._run_measurement_loop")
@patch("tools.check_pan_telemetry_only._open_serial")
def test_run_invalid_pre_cmd_json_aborts_before_serial_open(
    mock_open_serial: MagicMock, mock_loop: MagicMock
) -> None:
    with pytest.raises(SystemExit):
        run(
            "/dev/ttyAMA0",
            1.0,
            0.05,
            0.3,
            init_module=True,
            verbose_lines=False,
            pre_cmd_json="not valid json{",
        )

    mock_open_serial.assert_not_called()
    mock_loop.assert_not_called()


@patch("tools.check_pan_telemetry_only._open_serial")
def test_run_tx_log_captures_init_pre_cmd_and_query_phases(
    mock_open_serial: MagicMock,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: pathlib.Path,
) -> None:
    import tools.check_pan_telemetry_only as module

    clock = _FakeClock()
    monkeypatch.setattr(module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(module.time, "sleep", clock.sleep)

    ser = _fake_serial_context()
    ser.readline.side_effect = lambda: b'{"T":1001,"pan":1.0}\n'
    mock_open_serial.return_value = ser

    tx_log_path = tmp_path / "tx.jsonl"

    run(
        "/dev/ttyAMA0",
        0.01,
        0.05,
        0.3,
        init_module=True,
        verbose_lines=False,
        pre_cmd_json='{"T":123}',
        tx_log_jsonl_path=str(tx_log_path),
    )

    records = [json.loads(line) for line in tx_log_path.read_text().splitlines()]
    assert [r["phase"] for r in records] == ["init", "pre_cmd", "query"]
    assert records[0]["payload"] == {"T": 900, "main": 2, "module": 2}
    assert records[1]["payload"] == {"T": 123}
    assert records[2]["payload"] == {"T": 130}


@patch("tools.check_pan_telemetry_only._open_serial")
def test_run_default_flags_preserve_write_sequence(
    mock_open_serial: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    import tools.check_pan_telemetry_only as module

    clock = _FakeClock()
    monkeypatch.setattr(module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(module.time, "sleep", clock.sleep)

    ser = _fake_serial_context()
    ser.readline.side_effect = lambda: b'{"T":1001,"pan":1.0}\n'
    mock_open_serial.return_value = ser

    run("/dev/ttyAMA0", 0.12, 0.05, 0.3, init_module=True, verbose_lines=False)

    assert ser.write.call_args_list == [
        call(b'{"T":900,"main":2,"module":2}\n'),
        call(b'{"T":130}\n'),
        call(b'{"T":130}\n'),
        call(b'{"T":130}\n'),
    ]
