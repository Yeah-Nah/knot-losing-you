"""Passively listen to the Waveshare UGV Rover ESP32 serial output.

The ESP32 sub-controller on many firmware versions broadcasts telemetry
continuously without needing a poll command. This script just listens and
prints everything received, so you can see what data (including battery
voltage) the firmware actually sends.

Usage
-----
    python check_battery.py
    python check_battery.py --port /dev/serial0   # Pi 4B
    python check_battery.py --duration 10          # listen for 10 seconds
"""

from __future__ import annotations

import argparse
import json
import time

import serial
from loguru import logger

_VOLTAGE_KEYS = ("v", "V", "battery", "bat", "voltage", "vol", "Vbat", "vbat")
_WARN_LOW = 10.5
_FULL = 12.6

# Alternative T command numbers to try if passive listening yields nothing.
_PROBE_COMMANDS = [
    {"T": 105},
    {"T": 106},
    {"T": 1003},
    {"T": 1},
]


def _interpret_voltage(volts: float) -> str:
    if volts >= _FULL:
        return "FULL"
    if volts >= 11.1:
        return "OK"
    if volts >= _WARN_LOW:
        return "LOW — consider charging soon"
    return "CRITICAL — BMS cutoff likely imminent"


def _print_response(tag: str, data: dict) -> None:
    voltage = next((data[k] for k in _VOLTAGE_KEYS if k in data), None)
    if voltage is not None:
        status = _interpret_voltage(float(voltage))
        logger.success(f"{tag} Battery: {voltage} V — {status}  {data}")
    else:
        logger.info(f"{tag} {data}")


def listen(port: str, duration: float) -> None:
    logger.info(f"Opening {port} at 115200 baud — listening for {duration}s...")
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

        # Phase 1: passive listen — see what the ESP32 broadcasts on its own.
        logger.info("--- Phase 1: passive listen (no commands sent) ---")
        passive_lines = 0
        deadline = time.monotonic() + min(duration / 2, 5.0)
        while time.monotonic() < deadline:
            raw = ser.readline().decode("utf-8", errors="replace").strip()
            if not raw:
                continue
            passive_lines += 1
            try:
                _print_response("[passive]", json.loads(raw))
            except json.JSONDecodeError:
                logger.info(f"[passive] raw: {raw!r}")

        if passive_lines == 0:
            logger.warning("No passive output — ESP32 requires polling.")

        # Phase 2: send chassis init then probe alternative command numbers.
        logger.info("--- Phase 2: chassis init + probe commands ---")
        ser.write(json.dumps({"T": 900, "main": 2, "module": 0}, separators=(",", ":")).encode() + b"\n")
        time.sleep(0.3)
        ser.reset_input_buffer()

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
                    _print_response(f"[T:{cmd['T']}]", json.loads(raw))
                except json.JSONDecodeError:
                    logger.info(f"[T:{cmd['T']}] raw: {raw!r}")
            if not got:
                logger.warning(f"[T:{cmd['T']}] no response")

        # Phase 3: remaining time passive listen (catches delayed or async responses).
        logger.info("--- Phase 3: passive listen after probing ---")
        deadline = time.monotonic() + max(duration / 2, 3.0)
        while time.monotonic() < deadline:
            raw = ser.readline().decode("utf-8", errors="replace").strip()
            if not raw:
                continue
            try:
                _print_response("[post-probe]", json.loads(raw))
            except json.JSONDecodeError:
                logger.info(f"[post-probe] raw: {raw!r}")
        except serial.SerialException as exc:
            logger.error(
                f"Serial read failed mid-session: {exc}\n"
                "The port may have been grabbed by another process (e.g. ugv_rpi).\n"
                "Stop it with: sudo systemctl stop ugv_rpi"
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Listen to Waveshare UGV Rover ESP32 serial output.")
    parser.add_argument("--port", default="/dev/ttyAMA0")
    parser.add_argument("--duration", type=float, default=8.0, help="Total listen duration in seconds (default: 8)")
    args = parser.parse_args()
    listen(args.port, args.duration)


if __name__ == "__main__":
    main()
