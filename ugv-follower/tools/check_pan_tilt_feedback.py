"""Probe whether the Waveshare UGV Rover ESP32 firmware exposes pan servo position.

Commands the pan-tilt module to a known angle via T=133, then passively listens
for JSON telemetry. A second phase probes candidate T-codes one at a time and
logs every response. A final summary reports whether any received JSON field
resembles angle, position, or pan feedback data.

Usage
-----
    ugv-check-pan-tilt-feedback
    ugv-check-pan-tilt-feedback --port /dev/serial0    # Pi 4B
    ugv-check-pan-tilt-feedback --angle 30             # command to 30°
    ugv-check-pan-tilt-feedback --duration 15          # listen for 15 seconds

    # Or via python -m:
    python -m tools.check_pan_tilt_feedback
"""

from __future__ import annotations

import argparse
import json
import time
from typing import Any

import serial
from loguru import logger

from ugv_follower.utils.camera_preflight import ensure_character_device_available

_POSITION_KEYS = ("x", "X", "pan", "angle", "pos", "position", "servo", "feedback", "deg")
_PROBE_COMMANDS = [
    {"T": 105},
    {"T": 106},
    {"T": 130},
    {"T": 131},
    {"T": 132},
    {"T": 134},
]


def _print_response(tag: str, data: dict[str, Any]) -> bool:
    """Log *data* and return True if any field resembles pan position."""
    position = next((data[k] for k in _POSITION_KEYS if k in data), None)
    if position is not None:
        key = next(k for k in _POSITION_KEYS if k in data)
        logger.success(f"{tag} Pan position field '{key}': {position}  {data}")
        return True
    logger.debug(f"{tag} {data}")
    return False


def probe(port: str, angle_deg: float, duration: float) -> None:
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

            # Initialise chassis with pan-tilt module (module=2).
            ser.write(
                json.dumps(
                    {"T": 900, "main": 2, "module": 2}, separators=(",", ":")
                ).encode()
                + b"\n"
            )
            time.sleep(0.3)
            ser.reset_input_buffer()

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
                    position_found |= _print_response("[passive]", json.loads(raw))
                except json.JSONDecodeError:
                    logger.debug(f"[passive] raw: {raw!r}")

            if passive_lines == 0:
                logger.warning("No passive output — ESP32 requires polling.")

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
                        position_found |= _print_response(
                            f"[T:{cmd['T']}]", json.loads(raw)
                        )
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
                    position_found |= _print_response("[post-probe]", json.loads(raw))
                except json.JSONDecodeError:
                    logger.debug(f"[post-probe] raw: {raw!r}")

            if position_found:
                logger.success("Pan servo position feedback IS available over serial.")
            else:
                logger.warning(
                    "No pan position data found. "
                    "Re-run with --duration 15 or check firmware version."
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
    args = parser.parse_args()
    probe(args.port, args.angle, args.duration)


if __name__ == "__main__":
    main()
