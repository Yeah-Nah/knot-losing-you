"""Probe ESP32 JSON for pan telemetry (`T=1001.pan`) on Waveshare UGV Rover.

The script commands pan with `T=133`, listens passively, then explicitly polls
`T=130` to fetch `T=1001` telemetry. It also probes related T-codes for context
while only treating `T=1001.pan` as valid pan position feedback.

T=1005 bus servo error packets are tracked throughout all phases. If `FeedBack()`
fails on the ESP32 for a servo ID, the firmware emits::

    {"T":1005,"id":<servo_id>,"status":0}

A stream of T=1005 packets confirms that `FeedBack()` is failing every cycle
(root cause of the constant -179.9560394 reading), pointing to a bus-level
problem: wrong servo ID, wiring fault, or baud mismatch on the servo bus.

Usage
-----
    # Run with defaults (port=/dev/ttyAMA0, angle=20, duration=8s)
    ugv-check-pan-tilt-feedback

    # Command different targets
    ugv-check-pan-tilt-feedback --angle 20
    ugv-check-pan-tilt-feedback --angle 60

    # Longer capture and alternate port
    ugv-check-pan-tilt-feedback --duration 15 --port /dev/serial0

    # Skip T=900 init to compare behavior with/without module re-init
    ugv-check-pan-tilt-feedback --angle 60 --no-init

    # Or via python -m:
    python -m tools.check_pan_tilt_feedback --angle 60 --no-init
"""

from __future__ import annotations

import argparse
import json
import time
from typing import Any

import serial
from loguru import logger

from ugv_follower.utils.camera_preflight import ensure_character_device_available

_PAN_TELEMETRY_T = 1001
_BUS_SERVO_ERROR_T = 1005
_PROBE_COMMANDS = [
    {"T": 105},
    {"T": 106},
    {"T": 130},
    {"T": 131},
    {"T": 132},
    {"T": 134},
]


def _extract_pan(data: dict[str, Any]) -> float | None:
    """Return pan in degrees only for telemetry payloads with T=1001."""
    if data.get("T") != _PAN_TELEMETRY_T:
        return None
    pan = data.get("pan")
    if isinstance(pan, (int, float)):
        return float(pan)
    return None


def _check_bus_servo_error(tag: str, data: dict[str, Any]) -> bool:
    """Log T=1005 bus servo errors. Returns True if this was a T=1005 error packet."""
    if data.get("T") != _BUS_SERVO_ERROR_T:
        return False
    if data.get("status") == 0:
        logger.error(
            f"{tag} BUS SERVO FEEDBACK FAILURE: FeedBack() returned -1 for "
            f"servo id={data.get('id')}. ESP32 cannot read servo position. "
            "Check: servo ID constant in firmware, bus wiring, servo power."
        )
    else:
        logger.warning(f"{tag} T=1005 servo status: {data}")
    return True


def _print_response(tag: str, data: dict[str, Any]) -> bool:
    """Log data and return True only for `T=1001` with numeric `pan`."""
    _check_bus_servo_error(tag, data)
    pan = _extract_pan(data)
    if pan is not None:
        logger.success(f"{tag} Pan telemetry `T=1001.pan`: {pan}  {data}")
        return True
    logger.debug(f"{tag} {data}")
    return False


def _poll_t130_for_pan(ser: serial.Serial, polls: int = 10, wait_s: float = 0.2) -> list[float]:
    """Poll `T=130` and return all sampled `T=1001.pan` values."""
    pans: list[float] = []
    for _ in range(polls):
        ser.write(b'{"T":130}\n')
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            raw = ser.readline().decode("utf-8", errors="replace").strip()
            if not raw:
                continue
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                logger.debug(f"[T:130-poll] raw: {raw!r}")
                continue

            pan = _extract_pan(data)
            if pan is not None:
                pans.append(pan)
                logger.success(f"[T:130-poll] pan={pan}")
            else:
                logger.debug(f"[T:130-poll] {data}")
    return pans


def probe(port: str, angle_deg: float, duration: float, init_module: bool) -> None:
    """Probe ESP32 serial output for pan servo position feedback.

    Parameters
    ----------
    port : str
        Serial port path.
    angle_deg : float
        Pan angle to command before listening (degrees).
    duration : float
        Total probe duration in seconds.
    """
    logger.info(f"Opening {port} at 115200 baud — probing for {duration}s...")
    try:
        ensure_character_device_available(port, device_label="Serial port")
    except RuntimeError as exc:
        logger.error(
            f"{exc}\n"
            "If another process holds the port (e.g. the ugv_rpi service), stop it first:\n"
            "    sudo systemctl stop ugv_rpi"
        )
        return
    try:
        ser_conn = serial.Serial(port, 115200, timeout=1)
    except serial.SerialException as exc:
        logger.error(
            f"Could not open {port}: {exc}\n"
            "If another process holds the port (e.g. the ugv_rpi service), stop it first:\n"
            "    sudo systemctl stop ugv_rpi"
        )
        return
    with ser_conn as ser:
        try:
            time.sleep(0.1)
            position_found = False
            bus_servo_errors: list[dict[str, Any]] = []

            if init_module:
                # Initialise chassis with pan-tilt module (module=2).
                ser.write(
                    json.dumps(
                        {"T": 900, "main": 2, "module": 2}, separators=(",", ":")
                    ).encode()
                    + b"\n"
                )
                time.sleep(0.3)
                ser.reset_input_buffer()

            # Ensure InfoPrint=1 so the firmware emits T=1005 on FeedBack() failure.
            # Per firmware comments, 1 is the default, but send explicitly to be sure.
            ser.write(b'{"T":605,"cmd":1}\n')
            logger.debug("Sent T=605 cmd=1 (InfoPrint enable)")
            time.sleep(0.1)

            # Command pan to known angle.
            pan_cmd = json.dumps(
                {"T": 133, "X": angle_deg, "Y": 0, "SPD": 0, "ACC": 0},
                separators=(",", ":"),
            )
            ser.write(pan_cmd.encode() + b"\n")
            logger.info(f"Pan commanded to {angle_deg}°: {pan_cmd}")
            time.sleep(0.3)

            # Phase 1: passive listen — see what the ESP32 broadcasts on its own.
            logger.debug("--- Phase 1: passive listen (no commands sent) ---")
            passive_lines = 0
            deadline = time.monotonic() + min(duration / 2, 5.0)
            while time.monotonic() < deadline:
                raw = ser.readline().decode("utf-8", errors="replace").strip()
                if not raw:
                    continue
                passive_lines += 1
                try:
                    data = json.loads(raw)
                    if data.get("T") == _BUS_SERVO_ERROR_T and data.get("status") == 0:
                        bus_servo_errors.append(data)
                    position_found |= _print_response("[passive]", data)
                except json.JSONDecodeError:
                    logger.debug(f"[passive] raw: {raw!r}")

            if passive_lines == 0:
                logger.warning("No passive output — ESP32 requires polling.")

            # Phase 1b: explicit poll for T=1001.pan snapshots.
            logger.debug("--- Phase 1b: explicit T=130 polling ---")
            polled = _poll_t130_for_pan(ser)
            if polled:
                position_found = True
                logger.info(
                    "T=130 pan samples: "
                    f"count={len(polled)} min={min(polled):.6f} max={max(polled):.6f}"
                )
            else:
                logger.warning("T=130 returned no T=1001.pan samples.")

            # Phase 2: probe candidate T-codes.
            logger.debug("--- Phase 2: probe T-codes ---")
            for cmd in _PROBE_COMMANDS:
                payload = json.dumps(cmd, separators=(",", ":"))
                ser.write(payload.encode() + b"\n")
                logger.debug(f"Sent: {payload}")
                time.sleep(0.3)
                got = False
                while ser.in_waiting:
                    raw = ser.readline().decode("utf-8", errors="replace").strip()
                    if not raw:
                        continue
                    got = True
                    try:
                        data = json.loads(raw)
                        if data.get("T") == _BUS_SERVO_ERROR_T and data.get("status") == 0:
                            bus_servo_errors.append(data)
                        position_found |= _print_response(f"[T:{cmd['T']}]", data)
                    except json.JSONDecodeError:
                        logger.debug(f"[T:{cmd['T']}] raw: {raw!r}")
                if not got:
                    logger.debug(f"[T:{cmd['T']}] no response")

            # Phase 3: passive listen after probing (catches delayed/async responses).
            logger.debug("--- Phase 3: passive listen after probing ---")
            deadline = time.monotonic() + max(duration / 2, 3.0)
            while time.monotonic() < deadline:
                raw = ser.readline().decode("utf-8", errors="replace").strip()
                if not raw:
                    continue
                try:
                    data = json.loads(raw)
                    if data.get("T") == _BUS_SERVO_ERROR_T and data.get("status") == 0:
                        bus_servo_errors.append(data)
                    position_found |= _print_response("[post-probe]", data)
                except json.JSONDecodeError:
                    logger.debug(f"[post-probe] raw: {raw!r}")

            # --- Summary ---
            if bus_servo_errors:
                ids = sorted({e.get("id") for e in bus_servo_errors})
                logger.error(
                    f"DIAGNOSIS: FeedBack() failed {len(bus_servo_errors)} time(s) across all phases "
                    f"for servo id(s) {ids}. "
                    "The ESP32 cannot read servo position — T=1001.pan will always be the "
                    "uninitialised default (-179.9560394). "
                    "Likely causes: wrong GIMBAL_PAN_ID constant in firmware, "
                    "servo bus wiring fault, or servo not powered."
                )
            elif position_found:
                logger.success("T=1001.pan feedback is available over serial. No T=1005 errors seen.")
            else:
                logger.warning(
                    "No T=1001.pan data found and no T=1005 bus errors detected. "
                    "Re-run with --duration 15, or the robot firmware may differ from the "
                    "ugv_base_ros repo (InfoPrint path may not be compiled in)."
                )

        except serial.SerialException as exc:
            logger.error(
                f"Serial read failed mid-session: {exc}\n"
                "The port may have been grabbed by another process (e.g. ugv_rpi).\n"
                "Stop it with: sudo systemctl stop ugv_rpi"
            )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Probe Waveshare UGV Rover ESP32 for pan servo position feedback."
    )
    parser.add_argument("--port", default="/dev/ttyAMA0")
    parser.add_argument("--angle", type=float, default=20.0, help="Pan angle to command (degrees)")
    parser.add_argument(
        "--duration",
        type=float,
        default=8.0,
        help="Total probe duration in seconds (default: 8)",
    )
    parser.add_argument(
        "--no-init",
        action="store_true",
        help="Skip sending T=900 init before probing.",
    )
    args = parser.parse_args()
    probe(args.port, args.angle, args.duration, init_module=not args.no_init)


if __name__ == "__main__":
    main()
