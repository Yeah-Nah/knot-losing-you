"""Scan the ST3215 servo bus to identify which IDs respond and which do not.

The ESP32 firmware periodically calls ``FeedBack()`` on every servo it knows
about — gimbal pan (ID 2), gimbal tilt (ID 1), and all four wheel drive servos
(IDs 11, 12, 13, 14, 15). When ``FeedBack()`` fails for any of those IDs the
firmware emits::

    {"T":1005,"id":<servo_id>,"status":0}

This script enables that diagnostic output (``T=605 cmd=1``), listens across
multiple telemetry cycles, and cross-references which servo groups succeed or
fail. The result distinguishes between two root causes that would both produce
a stuck ``T=1001.pan``:

**Wrong gimbal ID** — the physical pan servo is programmed to a different ID
than the firmware constant ``GIMBAL_PAN_ID=2``. Symptom: gimbal IDs (1, 2)
appear in T=1005 but drive IDs (11–15) do *not*, and ``T=1001.L``/``.R``
show real wheel odometry.

**Bus-wide RX failure** — the ESP32 UART direction pin is stuck in TX mode so
every ``FeedBack()`` call times out, regardless of servo ID. Symptom: gimbal
*and* drive IDs all appear in T=1005, and ``T=1001.L``/``.R`` are always zero.

Usage
-----
    # Default: /dev/ttyAMA0, 20 s listen window
    ugv-scan-servo-ids

    # Longer listen to catch more telemetry cycles
    ugv-scan-servo-ids --duration 30

    # Alternate port, skip T=900 init
    ugv-scan-servo-ids --port /dev/serial0 --no-init

    # Or via python -m:
    python -m tools.scan_servo_ids
"""

from __future__ import annotations

import argparse
import json
import time
from collections import defaultdict
from typing import Any

import serial
from loguru import logger

from ugv_follower.utils.camera_preflight import ensure_character_device_available

# Servo IDs the firmware is expected to scan.
_GIMBAL_IDS: frozenset[int] = frozenset({1, 2})
_DRIVE_IDS: frozenset[int] = frozenset({11, 12, 13, 14, 15})

_T_TELEMETRY = 1001
_T_BUS_ERROR = 1005

# How many T=130 polls to fire per second during the listen window.
_POLL_INTERVAL_S = 0.5


def _group_label(servo_id: int) -> str:
    if servo_id in _GIMBAL_IDS:
        return "gimbal"
    if servo_id in _DRIVE_IDS:
        return "drive"
    return "unknown"


def scan(port: str, duration: float, init_module: bool) -> None:
    """Listen to the ESP32 serial stream and classify servo bus health.

    Parameters
    ----------
    port:
        Serial port path (e.g. ``/dev/ttyAMA0``).
    duration:
        Total listen window in seconds. Use ≥20 s to catch several
        firmware telemetry cycles.
    init_module:
        Send ``T=900`` module init before scanning.
    """
    logger.info(f"Opening {port} at 115200 baud — scan window: {duration:.0f} s")
    try:
        ensure_character_device_available(port, device_label="Serial port")
    except RuntimeError as exc:
        logger.error(
            f"{exc}\n"
            "Stop the ugv_rpi service first if it holds the port:\n"
            "    sudo systemctl stop ugv_rpi"
        )
        return

    try:
        ser_conn = serial.Serial(port, 115200, timeout=0.5)
    except serial.SerialException as exc:
        logger.error(
            f"Could not open {port}: {exc}\n"
            "Stop the ugv_rpi service first if it holds the port:\n"
            "    sudo systemctl stop ugv_rpi"
        )
        return

    with ser_conn as ser:
        try:
            time.sleep(0.1)

            # --- Setup ---
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

            # Enable InfoPrint so the firmware emits T=1005 on every FeedBack() failure.
            ser.write(b'{"T":605,"cmd":1}\n')
            logger.debug("Sent T=605 cmd=1 (InfoPrint enabled)")
            time.sleep(0.1)

            # --- Listen window ---
            # Periodically fire T=130 to trigger fresh gimbal feedback reads
            # between passive listen bursts.
            failure_counts: dict[int, int] = defaultdict(int)  # id → T=1005 count
            telemetry_samples: list[dict[str, Any]] = []  # T=1001 payloads
            next_poll = time.monotonic()
            deadline = time.monotonic() + duration
            total_lines = 0

            logger.info("Listening…  (Ctrl-C to stop early)")

            while time.monotonic() < deadline:
                # Fire a T=130 poll on schedule.
                now = time.monotonic()
                if now >= next_poll:
                    ser.write(b'{"T":130}\n')
                    next_poll = now + _POLL_INTERVAL_S

                raw = ser.readline().decode("utf-8", errors="replace").strip()
                if not raw:
                    continue

                total_lines += 1
                try:
                    data = json.loads(raw)
                except json.JSONDecodeError:
                    logger.debug(f"[raw] {raw!r}")
                    continue

                t_val = data.get("T")

                if t_val == _T_BUS_ERROR and data.get("status") == 0:
                    sid = data.get("id")
                    if isinstance(sid, int):
                        failure_counts[sid] += 1
                        grp = _group_label(sid)
                        logger.warning(
                            f"T=1005  id={sid} ({grp})  "
                            f"[total failures for this id: {failure_counts[sid]}]"
                        )

                elif t_val == _T_TELEMETRY:
                    telemetry_samples.append(data)
                    logger.debug(f"T=1001  {data}")

                else:
                    logger.debug(f"[other] {data}")

        except KeyboardInterrupt:
            logger.info("Scan interrupted by user.")
        except serial.SerialException as exc:
            logger.error(
                f"Serial read failed: {exc}\n"
                "The port may have been grabbed by another process."
            )

    # --- Diagnosis ---
    _print_diagnosis(failure_counts, telemetry_samples)


def _print_diagnosis(
    failure_counts: dict[int, int],
    telemetry_samples: list[dict[str, Any]],
) -> None:
    logger.info("=" * 60)
    logger.info("SERVO BUS SCAN — RESULTS")
    logger.info("=" * 60)

    if not failure_counts:
        logger.success("No T=1005 bus errors observed.")
        if telemetry_samples:
            logger.success(
                f"{len(telemetry_samples)} T=1001 telemetry samples received. "
                "FeedBack() appears to be succeeding — pan value should be live."
            )
        else:
            logger.warning(
                "No T=1005 errors and no T=1001 samples. "
                "Either the scan window was too short, or InfoPrint is not compiled "
                "into this firmware build. Try --duration 30."
            )
        return

    failed_ids = sorted(failure_counts.keys())
    failed_gimbal = sorted(i for i in failed_ids if i in _GIMBAL_IDS)
    failed_drive = sorted(i for i in failed_ids if i in _DRIVE_IDS)
    failed_unknown = sorted(i for i in failed_ids if i not in _GIMBAL_IDS | _DRIVE_IDS)

    logger.error(f"Failed servo IDs  : {failed_ids}")
    if failed_gimbal:
        logger.error(f"  Gimbal IDs      : {failed_gimbal}  (firmware expects PAN=2, TILT=1)")
    if failed_drive:
        logger.error(f"  Drive IDs       : {failed_drive}  (wheel motors)")
    if failed_unknown:
        logger.warning(f"  Unknown IDs     : {failed_unknown}")

    for sid, count in sorted(failure_counts.items()):
        logger.error(
            f"  id={sid} ({_group_label(sid)})  failures={count}"
        )

    # Check drive servo telemetry (L/R odometry in T=1001).
    drive_active = False
    if telemetry_samples:
        l_values = [s.get("L", 0) for s in telemetry_samples if "L" in s]
        r_values = [s.get("R", 0) for s in telemetry_samples if "R" in s]
        if l_values or r_values:
            l_nonzero = any(v != 0 for v in l_values)
            r_nonzero = any(v != 0 for v in r_values)
            drive_active = l_nonzero or r_nonzero
            logger.info(
                f"T=1001 drive odometry — L nonzero: {l_nonzero}, R nonzero: {r_nonzero}"
            )

    logger.info("-" * 60)
    logger.info("DIAGNOSIS:")

    both_gimbal_and_drive_failed = bool(failed_gimbal) and bool(failed_drive)
    only_gimbal_failed = bool(failed_gimbal) and not failed_drive

    if both_gimbal_and_drive_failed:
        logger.error(
            "LIKELY CAUSE: Half-duplex RX/direction-pin failure.\n"
            "  Both gimbal and drive servo IDs are failing FeedBack(). "
            "This is a bus-wide problem — the UART direction pin is probably "
            "stuck in TX mode, preventing the ESP32 from receiving any servo "
            "response packet. Wrong IDs would cause only gimbal IDs to fail.\n"
            "  Next step: inspect the SCServo UART direction-pin handling in "
            "the firmware source, or scope the DE/RE pin during a FeedBack() call."
        )
    elif only_gimbal_failed:
        if drive_active:
            logger.warning(
                "LIKELY CAUSE: Wrong gimbal servo IDs.\n"
                f"  Drive servos are working (T=1001 L/R odometry active) but "
                f"gimbal IDs {failed_gimbal} are failing. The physical pan/tilt "
                "servos are probably programmed to IDs other than PAN=2, TILT=1.\n"
                "  Next step: use a servo ID programmer (or the Waveshare Debug "
                "tool / SCServo SDK) to read the actual IDs off the servos, then "
                "either reprogram the servos to ID 2/1 or update GIMBAL_PAN_ID "
                "and GIMBAL_TILT_ID in the firmware."
            )
        else:
            logger.warning(
                "POSSIBLE CAUSE: Wrong gimbal servo IDs (drive status unclear).\n"
                f"  Gimbal IDs {failed_gimbal} are failing. No drive servo failures "
                "were observed, but T=1001 L/R odometry was also not detected — "
                "the rover may not be moving so drive feedback may simply be zero. "
                "Consider running with the rover moving to confirm drive servos work."
            )
    elif failed_drive and not failed_gimbal:
        logger.warning(
            "Unusual: drive servo IDs are failing but gimbal IDs are not. "
            "Gimbal FeedBack() is succeeding — the pan value should be live. "
            "Drive servo IDs may be misconfigured in firmware."
        )

    if telemetry_samples:
        pan_vals = [s["pan"] for s in telemetry_samples if "pan" in s]
        if pan_vals:
            unique_pan = set(round(v, 4) for v in pan_vals)
            stuck = len(unique_pan) == 1
            logger.info(
                f"T=1001.pan samples: {len(pan_vals)} readings, "
                f"unique values: {len(unique_pan)}  {'<-- STUCK (FeedBack failing)' if stuck else '<-- CHANGING (FeedBack working)'}"
            )
            if stuck:
                logger.error(f"  Stuck pan value: {next(iter(unique_pan))}")
    else:
        logger.warning("No T=1001 telemetry samples captured — increase --duration.")

    logger.info("=" * 60)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Scan the ST3215 servo bus by observing T=1005 FeedBack() failure "
            "reports. Distinguishes wrong gimbal ID from bus-wide RX failure."
        )
    )
    parser.add_argument("--port", default="/dev/ttyAMA0", help="Serial port (default: /dev/ttyAMA0)")
    parser.add_argument(
        "--duration",
        type=float,
        default=20.0,
        help="Listen window in seconds (default: 20). Use ≥30 for noisy environments.",
    )
    parser.add_argument(
        "--no-init",
        action="store_true",
        help="Skip T=900 module init.",
    )
    args = parser.parse_args()
    scan(args.port, args.duration, init_module=not args.no_init)


if __name__ == "__main__":
    main()
