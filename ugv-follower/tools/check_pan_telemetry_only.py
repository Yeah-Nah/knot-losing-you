"""Telemetry-only diagnostic for pan servo `T=130` -> `T=1001.pan` queries.

The active pan-oscillation investigation (Issue 9) has narrowed the dominant
failure mode to pan telemetry queries timing out at a high rate, with fresh
samples arriving in bursts separated by multi-second gaps. This tool isolates
one variable: whether those gaps persist when `T=130` is the *only* traffic
on the serial bus.

During the timed measurement window this script sends nothing but recurring
`T=130` polls — no `T=1` drive commands, no `T=133` pan-command traffic. The
optional `T=900` module init is a one-time setup step before timing begins,
not part of the measurement loop. By removing concurrent bus traffic, the
result distinguishes between two branches of Issue 9:

- High success ratio, low latency here -> concurrent drive/pan-command
  traffic is a likely contributor to the gaps seen in production.
- Persistent long gaps even in isolation -> the cause is elsewhere (firmware
  response scheduling, read timing, or flush strategy), not bus contention.

Follow-up instrumentation (opt-in, all off by default) supports the
correlation experiment described in
`docs/engineering_theory/firmware_host_correlation_experiment.md`: host-side
edge timestamps, a post-timeout linger window, raw-line retention, optional
sequence-token correlation, and structured JSONL output for offline `rho`
(matched ratio), `g~` (median inter-success gap), and `C_uniq` (unique pan
value count) analysis. None of this changes default-mode behaviour — see
"Usage" below.

Usage
-----
    # Run with defaults (port=/dev/ttyAMA0, duration=30s, poll=0.05s, timeout=0.3s)
    ugv-check-pan-telemetry-only

    # Longer run with a tighter poll cadence
    ugv-check-pan-telemetry-only --duration 60 --poll-interval 0.02

    # Skip the T=900 init to compare behaviour with/without module re-init
    ugv-check-pan-telemetry-only --no-init

    # Skip the RX flush before each T=130 write, to test flush-strategy impact
    ugv-check-pan-telemetry-only --no-flush

    # Log every non-T=1001 line seen during the loop
    ugv-check-pan-telemetry-only --verbose-lines

    # Attach a host sequence token to each request and look for it echoed
    # back in T=1001 (requires firmware support; the field name/presence is
    # unconfirmed, so this degrades to "no match data" rather than erroring
    # when the field is absent)
    ugv-check-pan-telemetry-only --seq-token --token-field S

    # Extend the read window 50ms past --timeout to catch late replies,
    # without changing success/timeout accounting
    ugv-check-pan-telemetry-only --linger-ms 50

    # Retain every raw serial line and timestamp seen per query
    ugv-check-pan-telemetry-only --retain-raw-lines

    # Write the extended per-query schema as JSONL for offline rho/g~/C_uniq
    # analysis
    ugv-check-pan-telemetry-only --log-jsonl run.jsonl

    # Or via python -m:
    python -m tools.check_pan_telemetry_only
"""

from __future__ import annotations

import argparse
import json
import statistics
import time
from contextlib import nullcontext
from dataclasses import dataclass
from typing import Any, TextIO

import serial
from loguru import logger

from ugv_follower.utils.camera_preflight import ensure_character_device_available

_PAN_TELEMETRY_T = 1001
_PORT_BUSY_HINT = (
    "If another process holds the port (e.g. the ugv_rpi service), stop it first:\n"
    "    sudo systemctl stop ugv_rpi"
)


@dataclass(frozen=True)
class QueryResult:
    """Outcome of a single `T=130` query.

    Parameters
    ----------
    query_id : int
        Sequence number of this query within the measurement loop.
    send_time_monotonic : float
        ``time.monotonic()`` timestamp at which `T=130` was written.
    success : bool
        ``True`` if a valid `T=1001` numeric ``pan`` was read before timeout.
    latency_ms : float | None
        Milliseconds between send and a successful reply; ``None`` on timeout.
    pan_deg : float | None
        Measured pan angle in degrees; ``None`` on timeout.
    lines_read : int
        Total serial lines read while waiting for this query's reply.
    non_telemetry_lines : int
        Count of those lines that were not a valid `T=1001` pan payload.
    seq_token : int | None
        Host-generated token attached to the outgoing `T=130`, when
        ``--seq-token`` is enabled; ``None`` otherwise.
    echoed_token : int | None
        Token read back from `T=1001` under ``--token-field``, if present
        and integer-valued; ``None`` if correlation is not in use or the
        firmware did not echo a usable field.
    token_matched : bool | None
        ``True``/``False`` only when both ``seq_token`` and ``echoed_token``
        are present and comparable; ``None`` when there is no data to
        compare (correlation not in use, or no reply arrived) — this keeps
        "no data" distinct from a proven mismatch.
    first_byte_ts : float | None
        ``time.monotonic()`` timestamp at which the accepted reply line was
        observed by ``readline()``. The tool's I/O model reads whole lines
        synchronously, so this is the "line received" time rather than a
        literal first-serial-byte timestamp; it is still useful as the
        earliest available marker of reply arrival. ``None`` on timeout.
    parse_ts : float | None
        ``time.monotonic()`` timestamp at which the accepted line finished
        JSON-parsing; ``None`` on timeout.
    timeout_ts : float | None
        ``time.monotonic()`` value at which the nominal read window
        (``timeout_s`` past send) elapses. Recorded on both success and
        timeout paths so callers can compute deltas against it.
    linger_late_reply : bool
        ``True`` only if a valid reply was captured during an optional
        post-timeout linger window (``--linger-ms``); never flips
        ``success`` for the existing timeout accounting.
    raw_lines : tuple[tuple[float, str, bool], ...] | None
        ``(recv_ts, raw_text, parsed_ok)`` per line seen during the query
        window (including any linger window); populated only when
        ``--retain-raw-lines`` is set.
    """

    query_id: int
    send_time_monotonic: float
    success: bool
    latency_ms: float | None
    pan_deg: float | None
    lines_read: int
    non_telemetry_lines: int
    seq_token: int | None = None
    echoed_token: int | None = None
    token_matched: bool | None = None
    first_byte_ts: float | None = None
    parse_ts: float | None = None
    timeout_ts: float | None = None
    linger_late_reply: bool = False
    raw_lines: tuple[tuple[float, str, bool], ...] | None = None


@dataclass(frozen=True)
class SummaryStats:
    """End-of-run aggregate statistics over all queries in the loop.

    Parameters
    ----------
    total_queries : int
        Number of `T=130` queries sent during the measurement window.
    successful : int
        Number of queries that received a valid `T=1001` pan reply.
    timeout_count : int
        Number of queries that timed out without a valid reply.
    success_ratio : float
        ``successful / total_queries``; ``0.0`` when ``total_queries`` is 0.
    latency_min_ms : float | None
        Minimum successful-query latency in milliseconds.
    latency_median_ms : float | None
        Median successful-query latency in milliseconds.
    latency_p95_ms : float | None
        95th-percentile successful-query latency in milliseconds.
    latency_max_ms : float | None
        Maximum successful-query latency in milliseconds.
    longest_gap_s : float | None
        Largest gap in seconds between consecutive successful queries'
        ``send_time_monotonic`` values; ``None`` if fewer than 2 successes.
    matched_ratio : float | None
        Fraction of successful queries with ``token_matched=True`` (`rho`);
        ``None`` if ``--seq-token`` was not used this run.
    median_inter_success_gap_s : float | None
        Median gap in seconds between consecutive successes (`g~`),
        distinct from ``longest_gap_s``; ``None`` if fewer than 2 successes.
    unique_pan_count : int | None
        Count of distinct ``pan_deg`` values among successes (`C_uniq`);
        ``None`` if there were no successes.
    late_reply_after_timeout_count : int
        Count of queries with ``linger_late_reply=True``; ``0`` when
        ``--linger-ms`` was not used.
    """

    total_queries: int
    successful: int
    timeout_count: int
    success_ratio: float
    latency_min_ms: float | None
    latency_median_ms: float | None
    latency_p95_ms: float | None
    latency_max_ms: float | None
    longest_gap_s: float | None
    matched_ratio: float | None = None
    median_inter_success_gap_s: float | None = None
    unique_pan_count: int | None = None
    late_reply_after_timeout_count: int = 0


def _extract_pan(data: dict[str, Any]) -> float | None:
    """Return pan in degrees only for telemetry payloads with T=1001.

    Parameters
    ----------
    data : dict[str, Any]
        Parsed JSON payload from a single serial line.

    Returns
    -------
    float | None
        The numeric ``pan`` value if ``T == 1001`` and ``pan`` is numeric,
        otherwise ``None``.
    """
    if data.get("T") != _PAN_TELEMETRY_T:
        return None
    pan = data.get("pan")
    if isinstance(pan, (int, float)):
        return float(pan)
    return None


def _match_token(
    data: dict[str, Any], seq_token: int | None, token_field: str
) -> tuple[int | None, bool | None]:
    """Extract an echoed sequence token and compare it against *seq_token*.

    Parameters
    ----------
    data : dict[str, Any]
        Parsed JSON payload from a `T=1001` line.
    seq_token : int | None
        The token attached to the outgoing `T=130`, or ``None`` if token
        correlation is not in use for this query.
    token_field : str
        JSON field name to look up the echoed token under.

    Returns
    -------
    tuple[int | None, bool | None]
        ``(echoed_token, token_matched)``. Both are ``None`` when
        correlation is not in use, or when the field is absent or not
        integer-valued — keeping "no data" distinct from a proven mismatch.
    """
    if seq_token is None:
        return None, None
    echoed = data.get(token_field)
    if not isinstance(echoed, int):
        return None, None
    return echoed, echoed == seq_token


def _read_line_with_ts(ser: serial.Serial) -> tuple[str, float] | None:
    """Read one serial line and timestamp its arrival.

    Parameters
    ----------
    ser : serial.Serial
        Open serial connection to the UGV controller.

    Returns
    -------
    tuple[str, float] | None
        ``(raw_text, recv_ts)`` for a non-empty line, else ``None``.
    """
    raw = ser.readline().decode("utf-8", errors="replace").strip()
    if not raw:
        return None
    return raw, time.monotonic()


def _linger_for_late_reply(
    ser: serial.Serial,
    linger_deadline: float,
    retain_raw_lines: bool,
    raw_lines: list[tuple[float, str, bool]],
) -> tuple[bool, int, int]:
    """Keep reading past the nominal timeout, watching for a late pan reply.

    This never influences the caller's ``success``/``timeout``
    classification — it only reports whether a valid reply arrived in this
    extra window, for offline analysis of true-no-reply vs. late-reply.

    Parameters
    ----------
    ser : serial.Serial
        Open serial connection to the UGV controller.
    linger_deadline : float
        ``time.monotonic()`` value at which the linger window ends.
    retain_raw_lines : bool
        Whether to append seen lines to *raw_lines*.
    raw_lines : list[tuple[float, str, bool]]
        Mutable accumulator shared with the caller's nominal-window
        capture, appended to in place when *retain_raw_lines* is set.

    Returns
    -------
    tuple[bool, int, int]
        ``(late_reply_seen, extra_lines_read, extra_non_telemetry_lines)``.
    """
    late_reply_seen = False
    lines_read = 0
    non_telemetry_lines = 0
    while time.monotonic() < linger_deadline:
        line = _read_line_with_ts(ser)
        if line is None:
            continue
        raw, recv_ts = line
        lines_read += 1
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            non_telemetry_lines += 1
            if retain_raw_lines:
                raw_lines.append((recv_ts, raw, False))
            continue
        parsed_ok = _extract_pan(data) is not None
        if retain_raw_lines:
            raw_lines.append((recv_ts, raw, parsed_ok))
        if parsed_ok:
            late_reply_seen = True
            break
        non_telemetry_lines += 1
    return late_reply_seen, lines_read, non_telemetry_lines


def _send_and_await_pan(
    ser: serial.Serial,
    query_id: int,
    timeout_s: float,
    verbose_lines: bool,
    flush_before_query: bool = True,
    seq_token: int | None = None,
    token_field: str = "S",
    linger_s: float = 0.0,
    retain_raw_lines: bool = False,
) -> QueryResult:
    """Send one `T=130` query and wait for a valid `T=1001` pan reply.

    Mirrors ``UGVController.query_pan_deg`` semantics (optionally flush RX ->
    write `T=130` -> poll ``readline()`` until a valid pan value or timeout)
    while additionally tracking ``lines_read`` and ``non_telemetry_lines``,
    which ``query_pan_deg`` does not expose to its caller.

    With all optional instrumentation left at its default, behaviour and
    outward serial traffic are unchanged from the pre-instrumentation tool.

    Parameters
    ----------
    ser : serial.Serial
        Open serial connection to the UGV controller.
    query_id : int
        Sequence number to attach to the returned result.
    timeout_s : float
        Maximum time to wait for a valid reply before declaring a timeout.
    verbose_lines : bool
        When ``True``, log the raw content of every non-pan line at debug
        level.
    flush_before_query : bool, default True
        Whether to flush the RX input buffer before writing `T=130`.
    seq_token : int | None, default None
        When set, attach this token to the outgoing `T=130` under
        *token_field* and look for it echoed back in `T=1001`.
    token_field : str, default "S"
        JSON field name used for the sequence token, both outgoing and for
        reading the echo. The actual firmware field name is unconfirmed —
        see `docs/engineering_theory/firmware_host_correlation_experiment.md`.
    linger_s : float, default 0.0
        Extra time to keep listening past *timeout_s* for a late reply.
        ``0.0`` disables lingering. Never changes ``success``/``timeout``
        classification.
    retain_raw_lines : bool, default False
        When ``True``, retain every raw line and timestamp seen (including
        during any linger window) on the returned result.

    Returns
    -------
    QueryResult
        The measured outcome of this single query.
    """
    send_time = time.monotonic()
    if flush_before_query:
        ser.reset_input_buffer()
    if seq_token is None:
        ser.write(b'{"T":130}\n')
    else:
        payload = json.dumps(
            {"T": 130, token_field: seq_token}, separators=(",", ":")
        )
        ser.write((payload + "\n").encode())

    deadline = send_time + timeout_s
    timeout_ts = deadline
    lines_read = 0
    non_telemetry_lines = 0
    raw_lines: list[tuple[float, str, bool]] = []
    while time.monotonic() < deadline:
        line = _read_line_with_ts(ser)
        if line is None:
            continue
        raw, recv_ts = line
        lines_read += 1
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            non_telemetry_lines += 1
            if retain_raw_lines:
                raw_lines.append((recv_ts, raw, False))
            if verbose_lines:
                logger.debug(f"[query {query_id}] non-JSON line: {raw!r}")
            continue

        pan = _extract_pan(data)
        if pan is not None:
            parse_ts = time.monotonic()
            if retain_raw_lines:
                raw_lines.append((recv_ts, raw, True))
            echoed_token, token_matched = _match_token(data, seq_token, token_field)
            return QueryResult(
                query_id=query_id,
                send_time_monotonic=send_time,
                success=True,
                latency_ms=(parse_ts - send_time) * 1000.0,
                pan_deg=pan,
                lines_read=lines_read,
                non_telemetry_lines=non_telemetry_lines,
                seq_token=seq_token,
                echoed_token=echoed_token,
                token_matched=token_matched,
                first_byte_ts=recv_ts,
                parse_ts=parse_ts,
                timeout_ts=timeout_ts,
                linger_late_reply=False,
                raw_lines=tuple(raw_lines) if retain_raw_lines else None,
            )
        non_telemetry_lines += 1
        if retain_raw_lines:
            raw_lines.append((recv_ts, raw, True))
        if verbose_lines:
            logger.debug(f"[query {query_id}] non-pan line: {data}")

    linger_late_reply = False
    if linger_s > 0:
        linger_late_reply, extra_lines, extra_non_telemetry = _linger_for_late_reply(
            ser, deadline + linger_s, retain_raw_lines, raw_lines
        )
        lines_read += extra_lines
        non_telemetry_lines += extra_non_telemetry

    return QueryResult(
        query_id=query_id,
        send_time_monotonic=send_time,
        success=False,
        latency_ms=None,
        pan_deg=None,
        lines_read=lines_read,
        non_telemetry_lines=non_telemetry_lines,
        seq_token=seq_token,
        echoed_token=None,
        token_matched=None,
        first_byte_ts=None,
        parse_ts=None,
        timeout_ts=timeout_ts,
        linger_late_reply=linger_late_reply,
        raw_lines=tuple(raw_lines) if retain_raw_lines else None,
    )


def _sleep_until(target_monotonic: float) -> None:
    """Sleep only if *target_monotonic* is still in the future.

    Parameters
    ----------
    target_monotonic : float
        The ``time.monotonic()`` timestamp to sleep until.
    """
    remaining = target_monotonic - time.monotonic()
    if remaining > 0:
        time.sleep(remaining)


def _write_jsonl_record(jsonl_file: TextIO, result: QueryResult) -> None:
    """Append one JSON line for *result* to an open `--log-jsonl` file.

    Parameters
    ----------
    jsonl_file : TextIO
        Open, writable text file handle for the JSONL output.
    result : QueryResult
        The query outcome to serialise, using the full extended schema.
    """
    record = {
        "query_id": result.query_id,
        "send_time_monotonic": result.send_time_monotonic,
        "success": result.success,
        "latency_ms": result.latency_ms,
        "pan_deg": result.pan_deg,
        "lines_read": result.lines_read,
        "non_telemetry_lines": result.non_telemetry_lines,
        "seq_token": result.seq_token,
        "echoed_token": result.echoed_token,
        "token_matched": result.token_matched,
        "first_byte_ts": result.first_byte_ts,
        "parse_ts": result.parse_ts,
        "timeout_ts": result.timeout_ts,
        "linger_late_reply": result.linger_late_reply,
        "raw_lines": (
            [list(entry) for entry in result.raw_lines]
            if result.raw_lines is not None
            else None
        ),
    }
    jsonl_file.write(json.dumps(record) + "\n")


def _run_measurement_loop(
    ser: serial.Serial,
    duration_s: float,
    poll_interval_s: float,
    timeout_s: float,
    verbose_lines: bool,
    flush_before_query: bool = True,
    use_seq_token: bool = False,
    token_field: str = "S",
    linger_s: float = 0.0,
    retain_raw_lines: bool = False,
    jsonl_path: str | None = None,
) -> list[QueryResult]:
    """Send fixed-cadence `T=130` queries for *duration_s* and collect results.

    ``next_slot`` advances by exactly ``poll_interval_s`` every iteration with
    no resync and no catch-up. When queries return fast, `_sleep_until` waits
    out the remainder of the interval and cadence is genuinely
    ``poll_interval_s``. When queries time out, each iteration takes roughly
    ``timeout_s``, ``next_slot`` falls behind wall-clock, and `_sleep_until`
    stops sleeping at all — so the loop naturally settles into a
    timeout-dominated effective cadence instead of silently correcting for it.
    This drift is the behaviour this tool exists to observe, so it must not be
    corrected away.

    Parameters
    ----------
    ser : serial.Serial
        Open serial connection to the UGV controller.
    duration_s : float
        Total measurement window in seconds.
    poll_interval_s : float
        Target interval between the start of successive queries.
    timeout_s : float
        Per-query timeout passed to `_send_and_await_pan`.
    verbose_lines : bool
        Forwarded to `_send_and_await_pan`.
    flush_before_query : bool, default True
        Forwarded to `_send_and_await_pan`.
    use_seq_token : bool, default False
        When ``True``, attach an incrementing ``query_id``-based token
        (via `_send_and_await_pan`'s ``seq_token``) to every query.
    token_field : str, default "S"
        Forwarded to `_send_and_await_pan`.
    linger_s : float, default 0.0
        Forwarded to `_send_and_await_pan`.
    retain_raw_lines : bool, default False
        Forwarded to `_send_and_await_pan`.
    jsonl_path : str | None, default None
        When set, write one extended-schema JSON record per query to this
        path via `_write_jsonl_record`.

    Returns
    -------
    list[QueryResult]
        One entry per query sent during the window, in send order.
    """
    results: list[QueryResult] = []
    query_id = 0
    end_time = time.monotonic() + duration_s
    next_slot = time.monotonic()
    jsonl_cm = open(jsonl_path, "w", encoding="utf-8") if jsonl_path else nullcontext()
    with jsonl_cm as jsonl_file:
        while time.monotonic() < end_time:
            result = _send_and_await_pan(
                ser,
                query_id,
                timeout_s,
                verbose_lines,
                flush_before_query,
                seq_token=query_id if use_seq_token else None,
                token_field=token_field,
                linger_s=linger_s,
                retain_raw_lines=retain_raw_lines,
            )
            results.append(result)
            _log_query_result(result)
            if jsonl_file is not None:
                _write_jsonl_record(jsonl_file, result)
            query_id += 1
            next_slot += poll_interval_s
            _sleep_until(next_slot)
    return results


def _latency_percentiles(
    latencies: list[float],
) -> tuple[float | None, float | None, float | None, float | None]:
    """Compute (min, median, p95, max) latency in milliseconds.

    Parameters
    ----------
    latencies : list[float]
        Successful-query latencies in milliseconds.

    Returns
    -------
    tuple[float | None, float | None, float | None, float | None]
        ``(min, median, p95, max)``. All ``None`` for an empty list; all four
        equal for a single-value list.
    """
    if not latencies:
        return None, None, None, None
    if len(latencies) == 1:
        value = latencies[0]
        return value, value, value, value
    p95 = statistics.quantiles(latencies, n=100, method="inclusive")[94]
    return min(latencies), statistics.median(latencies), p95, max(latencies)


def _longest_gap_s(successes: list[QueryResult]) -> float | None:
    """Return the largest gap between consecutive successful queries.

    Parameters
    ----------
    successes : list[QueryResult]
        Successful query results, in send order.

    Returns
    -------
    float | None
        Largest ``send_time_monotonic`` difference between consecutive
        entries; ``None`` if fewer than 2 successes are present.
    """
    if len(successes) < 2:
        return None
    gaps = [
        b.send_time_monotonic - a.send_time_monotonic
        for a, b in zip(successes, successes[1:])
    ]
    return max(gaps)


def _median_inter_success_gap_s(successes: list[QueryResult]) -> float | None:
    """Return the median gap between consecutive successful queries (`g~`).

    Parameters
    ----------
    successes : list[QueryResult]
        Successful query results, in send order.

    Returns
    -------
    float | None
        Median ``send_time_monotonic`` difference between consecutive
        entries; ``None`` if fewer than 2 successes are present.
    """
    if len(successes) < 2:
        return None
    gaps = [
        b.send_time_monotonic - a.send_time_monotonic
        for a, b in zip(successes, successes[1:])
    ]
    return statistics.median(gaps)


def _unique_pan_count(successes: list[QueryResult]) -> int | None:
    """Return the count of distinct ``pan_deg`` values among successes (`C_uniq`).

    Parameters
    ----------
    successes : list[QueryResult]
        Successful query results.

    Returns
    -------
    int | None
        Count of distinct ``pan_deg`` values; ``None`` if there were no
        successes.
    """
    if not successes:
        return None
    return len({r.pan_deg for r in successes})


def _matched_ratio(
    results: list[QueryResult], successes: list[QueryResult]
) -> float | None:
    """Return the fraction of successes with a proven token match (`rho`).

    Parameters
    ----------
    results : list[QueryResult]
        All query results collected during the measurement loop.
    successes : list[QueryResult]
        The subset of *results* with ``success=True``.

    Returns
    -------
    float | None
        ``None`` if no query in *results* used a ``seq_token`` (token
        correlation was not in use this run); ``0.0`` if it was in use but
        there were no successes; otherwise the matched fraction.
    """
    if not any(r.seq_token is not None for r in results):
        return None
    if not successes:
        return 0.0
    matched = sum(1 for r in successes if r.token_matched is True)
    return matched / len(successes)


def _compute_summary(results: list[QueryResult]) -> SummaryStats:
    """Reduce a list of query results into end-of-run summary statistics.

    Parameters
    ----------
    results : list[QueryResult]
        All query results collected during the measurement loop.

    Returns
    -------
    SummaryStats
        Aggregate counts, latency percentiles, and correlation/gap/diversity
        metrics.
    """
    total = len(results)
    successes = [r for r in results if r.success]
    timeouts = total - len(successes)
    ratio = len(successes) / total if total else 0.0
    latencies = [r.latency_ms for r in successes if r.latency_ms is not None]
    lat_min, lat_median, lat_p95, lat_max = _latency_percentiles(latencies)
    return SummaryStats(
        total_queries=total,
        successful=len(successes),
        timeout_count=timeouts,
        success_ratio=ratio,
        latency_min_ms=lat_min,
        latency_median_ms=lat_median,
        latency_p95_ms=lat_p95,
        latency_max_ms=lat_max,
        longest_gap_s=_longest_gap_s(successes),
        matched_ratio=_matched_ratio(results, successes),
        median_inter_success_gap_s=_median_inter_success_gap_s(successes),
        unique_pan_count=_unique_pan_count(successes),
        late_reply_after_timeout_count=sum(1 for r in results if r.linger_late_reply),
    )


def _log_query_result(result: QueryResult) -> None:
    """Log a single per-query outcome line with all required fields.

    Extra debug-level lines are logged only when the corresponding
    instrumentation produced data, so default-mode output is unchanged.

    Parameters
    ----------
    result : QueryResult
        The query outcome to log.
    """
    if result.success:
        logger.success(
            "query_id={} send_t={:.3f} outcome=success latency_ms={:.2f} "
            "pan_deg={:.4f} lines_read={} non_telemetry_lines={}",
            result.query_id,
            result.send_time_monotonic,
            result.latency_ms,
            result.pan_deg,
            result.lines_read,
            result.non_telemetry_lines,
        )
    else:
        logger.warning(
            "query_id={} send_t={:.3f} outcome=timeout latency_ms=n/a "
            "pan_deg=n/a lines_read={} non_telemetry_lines={}",
            result.query_id,
            result.send_time_monotonic,
            result.lines_read,
            result.non_telemetry_lines,
        )
    if result.token_matched is not None:
        logger.debug(
            "query_id={} seq_token={} echoed_token={} token_matched={}",
            result.query_id,
            result.seq_token,
            result.echoed_token,
            result.token_matched,
        )
    if result.linger_late_reply:
        logger.debug(
            "query_id={} late reply captured during linger window",
            result.query_id,
        )


def _log_summary(stats: SummaryStats, show_extended: bool = False) -> None:
    """Log the end-of-run console summary block.

    Parameters
    ----------
    stats : SummaryStats
        Aggregate statistics computed by `_compute_summary`.
    show_extended : bool, default False
        When ``True``, also print the correlation/gap/diversity metrics
        (`rho`, `g~`, `C_uniq`, late-reply count). Kept off by default so
        default-mode console output is byte-identical to the pre-refactor
        tool regardless of what `_compute_summary` was able to derive from
        existing fields.
    """
    logger.info("=" * 60)
    logger.info("TELEMETRY-ONLY PAN QUERY — SUMMARY")
    logger.info("=" * 60)
    logger.info(f"Total queries    : {stats.total_queries}")
    logger.info(f"Successful       : {stats.successful}")
    logger.info(f"Timeouts         : {stats.timeout_count}")
    logger.info(f"Success ratio    : {stats.success_ratio:.1%}")
    if stats.latency_min_ms is not None:
        logger.info(
            "Latency ms (min/median/p95/max): "
            f"{stats.latency_min_ms:.2f} / {stats.latency_median_ms:.2f} / "
            f"{stats.latency_p95_ms:.2f} / {stats.latency_max_ms:.2f}"
        )
    else:
        logger.info("Latency ms (min/median/p95/max): n/a (no successful queries)")
    if stats.longest_gap_s is not None:
        logger.info(f"Longest gap between successes: {stats.longest_gap_s:.3f}s")
    else:
        logger.info("Longest gap between successes: n/a (fewer than 2 successes)")
    if show_extended:
        _log_extended_summary(stats)
    logger.info("=" * 60)


def _log_extended_summary(stats: SummaryStats) -> None:
    """Log the correlation/gap/diversity metric lines, when data is present.

    Parameters
    ----------
    stats : SummaryStats
        Aggregate statistics computed by `_compute_summary`.
    """
    if stats.matched_ratio is not None:
        logger.info(f"Matched ratio (rho)           : {stats.matched_ratio:.1%}")
    if stats.median_inter_success_gap_s is not None:
        logger.info(
            "Median inter-success gap (g~) : "
            f"{stats.median_inter_success_gap_s:.3f}s"
        )
    if stats.unique_pan_count is not None:
        logger.info(f"Unique pan values (C_uniq)    : {stats.unique_pan_count}")
    if stats.late_reply_after_timeout_count:
        logger.info(
            "Late replies caught in linger : "
            f"{stats.late_reply_after_timeout_count}"
        )


def _open_serial(port: str) -> serial.Serial | None:
    """Run device preflight then open the serial port.

    Parameters
    ----------
    port : str
        Serial port path (e.g. ``/dev/ttyAMA0``).

    Returns
    -------
    serial.Serial | None
        The open connection, or ``None`` if preflight or open failed (with
        the failure already logged).
    """
    try:
        ensure_character_device_available(port, device_label="Serial port")
    except RuntimeError as exc:
        logger.error(f"{exc}\n{_PORT_BUSY_HINT}")
        return None
    try:
        return serial.Serial(port, 115200, timeout=0.05)
    except serial.SerialException as exc:
        logger.error(f"Could not open {port}: {exc}\n{_PORT_BUSY_HINT}")
        return None


def _maybe_init(ser: serial.Serial, init_module: bool) -> None:
    """Optionally send `T=900` module init, then clear the startup buffer.

    Parameters
    ----------
    ser : serial.Serial
        Open serial connection to the UGV controller.
    init_module : bool
        Whether to send `T=900` (UGV Rover + pan-tilt module init) before
        clearing the buffer.
    """
    if init_module:
        ser.write(
            json.dumps(
                {"T": 900, "main": 2, "module": 2}, separators=(",", ":")
            ).encode()
            + b"\n"
        )
        logger.debug("Sent T=900 (module init, UGV Rover + pan-tilt)")
        time.sleep(0.3)
    ser.reset_input_buffer()


def run(
    port: str,
    duration_s: float,
    poll_interval_s: float,
    timeout_s: float,
    init_module: bool,
    verbose_lines: bool,
    flush_before_query: bool = True,
    use_seq_token: bool = False,
    token_field: str = "S",
    linger_s: float = 0.0,
    retain_raw_lines: bool = False,
    log_jsonl_path: str | None = None,
) -> None:
    """Run the telemetry-only measurement window and print the summary.

    Parameters
    ----------
    port : str
        Serial port path.
    duration_s : float
        Total measurement window in seconds.
    poll_interval_s : float
        Target interval between the start of successive `T=130` queries.
    timeout_s : float
        Per-query timeout in seconds.
    init_module : bool
        Send `T=900` module init before timing begins.
    verbose_lines : bool
        Log every non-`T=1001` line seen during the loop.
    flush_before_query : bool, default True
        Whether to flush the RX input buffer before each `T=130` write.
    use_seq_token : bool, default False
        Attach an incrementing host sequence token to every query and look
        for it echoed back in `T=1001`.
    token_field : str, default "S"
        JSON field name used for the sequence token.
    linger_s : float, default 0.0
        Extra time to keep listening past *timeout_s* for a late reply.
    retain_raw_lines : bool, default False
        Retain every raw serial line and timestamp seen per query.
    log_jsonl_path : str | None, default None
        When set, write one extended-schema JSON record per query to this
        path.
    """
    logger.info(
        f"Opening {port} at 115200 baud — telemetry-only window: "
        f"{duration_s:.1f}s, poll={poll_interval_s:.3f}s, timeout={timeout_s:.3f}s"
    )
    ser_conn = _open_serial(port)
    if ser_conn is None:
        return
    with ser_conn as ser:
        time.sleep(0.1)
        _maybe_init(ser, init_module)
        try:
            results = _run_measurement_loop(
                ser,
                duration_s,
                poll_interval_s,
                timeout_s,
                verbose_lines,
                flush_before_query,
                use_seq_token=use_seq_token,
                token_field=token_field,
                linger_s=linger_s,
                retain_raw_lines=retain_raw_lines,
                jsonl_path=log_jsonl_path,
            )
        except serial.SerialException as exc:
            logger.error(
                f"Serial read failed mid-session: {exc}\n"
                "The port may have been grabbed by another process (e.g. ugv_rpi).\n"
                "Stop it with: sudo systemctl stop ugv_rpi"
            )
            return
    show_extended = (
        use_seq_token or linger_s > 0 or retain_raw_lines or log_jsonl_path is not None
    )
    _log_summary(_compute_summary(results), show_extended=show_extended)


def main() -> None:
    """Parse CLI arguments and run the telemetry-only diagnostic.

    CLI options
    -----------
    --port : str, default ``/dev/ttyAMA0``
        Serial port path.
    --duration : float, default ``30.0``
        Measurement window in seconds.
    --poll-interval : float, default ``0.05``
        Target interval between the start of successive `T=130` queries.
    --timeout : float, default ``0.3``
        Per-query timeout in seconds.
    --no-init : flag
        Skip the `T=900` module init before timing begins.
    --no-flush : flag
        Skip the RX buffer flush before each `T=130` write.
    --verbose-lines : flag
        Log every non-`T=1001` line seen during the loop.
    --seq-token : flag
        Attach an incrementing host sequence token to every query and look
        for it echoed back in `T=1001` (see `--token-field`).
    --token-field : str, default ``"S"``
        JSON field name for the sequence token; unconfirmed against
        firmware, see
        `docs/engineering_theory/firmware_host_correlation_experiment.md`.
    --linger-ms : float, default ``0.0``
        Extra listen time in milliseconds past `--timeout`, to catch late
        replies without changing success/timeout accounting.
    --retain-raw-lines : flag
        Retain every raw serial line and timestamp seen per query.
    --log-jsonl : str, default ``None``
        Write one extended-schema JSON record per query to this path.
    """
    parser = argparse.ArgumentParser(
        description=(
            "Send only T=130 pan telemetry queries at a fixed cadence for a "
            "timed window and report success ratio, latency stats, and gap "
            "behaviour — isolating whether T=130/T=1001 gaps persist without "
            "concurrent drive/pan-command traffic."
        )
    )
    parser.add_argument("--port", default="/dev/ttyAMA0", help="Serial port path.")
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Measurement window in seconds (default: 30.0).",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=0.05,
        help="Target interval between queries in seconds (default: 0.05).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.3,
        help="Per-query timeout in seconds (default: 0.3).",
    )
    parser.add_argument(
        "--no-init",
        action="store_true",
        help="Skip sending T=900 module init before timing begins.",
    )
    parser.add_argument(
        "--no-flush",
        action="store_true",
        help="Skip the RX buffer flush before each T=130 write.",
    )
    parser.add_argument(
        "--verbose-lines",
        action="store_true",
        help="Log every non-T=1001 line seen during the measurement loop.",
    )
    parser.add_argument(
        "--seq-token",
        action="store_true",
        help=(
            "Attach an incrementing host sequence token to each T=130 "
            "request and look for it echoed back in T=1001, to prove "
            "same-cycle correlation. Requires firmware support; degrades "
            "gracefully to 'no match data' when the field is absent."
        ),
    )
    parser.add_argument(
        "--token-field",
        default="S",
        help=(
            "JSON field name for the sequence token, outgoing and echoed "
            "(default: 'S'). Unconfirmed against firmware — see "
            "docs/engineering_theory/firmware_host_correlation_experiment.md."
        ),
    )
    parser.add_argument(
        "--linger-ms",
        type=float,
        default=0.0,
        help=(
            "Extra listen time in milliseconds past --timeout, to catch "
            "late replies without changing success/timeout accounting "
            "(default: 0 = off). Keep well under --poll-interval to avoid "
            "bleeding into the next query's read window."
        ),
    )
    parser.add_argument(
        "--retain-raw-lines",
        action="store_true",
        help=(
            "Retain every raw serial line and timestamp seen per query "
            "(opt-in; unbounded retention, so avoid on very long runs)."
        ),
    )
    parser.add_argument(
        "--log-jsonl",
        default=None,
        metavar="PATH",
        help="Write one extended-schema JSON record per query to PATH.",
    )
    args = parser.parse_args()
    run(
        args.port,
        args.duration,
        args.poll_interval,
        args.timeout,
        init_module=not args.no_init,
        verbose_lines=args.verbose_lines,
        flush_before_query=not args.no_flush,
        use_seq_token=args.seq_token,
        token_field=args.token_field,
        linger_s=args.linger_ms / 1000.0,
        retain_raw_lines=args.retain_raw_lines,
        log_jsonl_path=args.log_jsonl,
    )


if __name__ == "__main__":
    main()
