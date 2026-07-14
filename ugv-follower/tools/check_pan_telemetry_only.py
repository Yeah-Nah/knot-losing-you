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

    # Or via python -m:
    python -m tools.check_pan_telemetry_only
"""

from __future__ import annotations

import argparse
import json
import statistics
import time
from dataclasses import dataclass
from typing import Any

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
    """

    query_id: int
    send_time_monotonic: float
    success: bool
    latency_ms: float | None
    pan_deg: float | None
    lines_read: int
    non_telemetry_lines: int


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


def _send_and_await_pan(
    ser: serial.Serial,
    query_id: int,
    timeout_s: float,
    verbose_lines: bool,
    flush_before_query: bool = True,
) -> QueryResult:
    """Send one `T=130` query and wait for a valid `T=1001` pan reply.

    Mirrors ``UGVController.query_pan_deg`` semantics (optionally flush RX ->
    write `T=130` -> poll ``readline()`` until a valid pan value or timeout)
    while additionally tracking ``lines_read`` and ``non_telemetry_lines``,
    which ``query_pan_deg`` does not expose to its caller.

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

    Returns
    -------
    QueryResult
        The measured outcome of this single query.
    """
    send_time = time.monotonic()
    if flush_before_query:
        ser.reset_input_buffer()
    ser.write(b'{"T":130}\n')

    deadline = send_time + timeout_s
    lines_read = 0
    non_telemetry_lines = 0
    while time.monotonic() < deadline:
        raw = ser.readline().decode("utf-8", errors="replace").strip()
        if not raw:
            continue
        lines_read += 1
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            non_telemetry_lines += 1
            if verbose_lines:
                logger.debug(f"[query {query_id}] non-JSON line: {raw!r}")
            continue

        pan = _extract_pan(data)
        if pan is not None:
            latency_ms = (time.monotonic() - send_time) * 1000.0
            return QueryResult(
                query_id=query_id,
                send_time_monotonic=send_time,
                success=True,
                latency_ms=latency_ms,
                pan_deg=pan,
                lines_read=lines_read,
                non_telemetry_lines=non_telemetry_lines,
            )
        non_telemetry_lines += 1
        if verbose_lines:
            logger.debug(f"[query {query_id}] non-pan line: {data}")

    return QueryResult(
        query_id=query_id,
        send_time_monotonic=send_time,
        success=False,
        latency_ms=None,
        pan_deg=None,
        lines_read=lines_read,
        non_telemetry_lines=non_telemetry_lines,
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


def _run_measurement_loop(
    ser: serial.Serial,
    duration_s: float,
    poll_interval_s: float,
    timeout_s: float,
    verbose_lines: bool,
    flush_before_query: bool = True,
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

    Returns
    -------
    list[QueryResult]
        One entry per query sent during the window, in send order.
    """
    results: list[QueryResult] = []
    query_id = 0
    end_time = time.monotonic() + duration_s
    next_slot = time.monotonic()
    while time.monotonic() < end_time:
        result = _send_and_await_pan(
            ser, query_id, timeout_s, verbose_lines, flush_before_query
        )
        results.append(result)
        _log_query_result(result)
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


def _compute_summary(results: list[QueryResult]) -> SummaryStats:
    """Reduce a list of query results into end-of-run summary statistics.

    Parameters
    ----------
    results : list[QueryResult]
        All query results collected during the measurement loop.

    Returns
    -------
    SummaryStats
        Aggregate counts, latency percentiles, and longest success gap.
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
    )


def _log_query_result(result: QueryResult) -> None:
    """Log a single per-query outcome line with all required fields.

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


def _log_summary(stats: SummaryStats) -> None:
    """Log the end-of-run console summary block.

    Parameters
    ----------
    stats : SummaryStats
        Aggregate statistics computed by `_compute_summary`.
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
    logger.info("=" * 60)


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
            )
        except serial.SerialException as exc:
            logger.error(
                f"Serial read failed mid-session: {exc}\n"
                "The port may have been grabbed by another process (e.g. ugv_rpi).\n"
                "Stop it with: sudo systemctl stop ugv_rpi"
            )
            return
    _log_summary(_compute_summary(results))


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
    args = parser.parse_args()
    run(
        args.port,
        args.duration,
        args.poll_interval,
        args.timeout,
        init_module=not args.no_init,
        verbose_lines=args.verbose_lines,
        flush_before_query=not args.no_flush,
    )


if __name__ == "__main__":
    main()
