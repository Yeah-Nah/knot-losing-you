"""Read and print battery voltage from the Waveshare UGV Rover ESP32 sub-controller.

Sends a T:105 telemetry request over serial and prints every response received.
Run with the rover powered on and connected to the Pi's UART.

Usage
-----
    python check_battery.py

    # Override port (Pi 4B uses /dev/serial0):
    python check_battery.py --port /dev/serial0

    # Change number of samples and interval:
    python check_battery.py --samples 10 --interval 1.0
"""

from __future__ import annotations

import argparse
import json
import time

import serial
from loguru import logger


# Voltage keys the Waveshare ESP32 firmware may use in its JSON responses.
_VOLTAGE_KEYS = ("v", "V", "battery", "bat", "voltage")

# Healthy voltage range for a 3-cell (3S) lithium pack.
_WARN_LOW = 10.5   # V — approaching BMS cutoff territory
_FULL = 12.6       # V — fully charged


def _interpret_voltage(volts: float) -> str:
    if volts >= _FULL:
        return "FULL"
    if volts >= 11.1:
        return "OK"
    if volts >= _WARN_LOW:
        return "LOW — consider charging soon"
    return "CRITICAL — BMS cutoff likely imminent"


def check_battery(port: str, samples: int, interval: float) -> None:
    logger.info(f"Opening {port} at 115200 baud...")
    with serial.Serial(port, 115200, timeout=2) as ser:
        time.sleep(0.1)  # Allow ESP32 to settle after port open

        # Chassis init is required after power-on before other commands are accepted.
        init_cmd = json.dumps({"T": 900, "main": 2, "module": 0}, separators=(",", ":"))
        ser.write(init_cmd.encode() + b"\n")
        logger.debug(f"Sent chassis init: {init_cmd}")
        time.sleep(0.2)
        ser.reset_input_buffer()  # Discard any init response / boot noise

        for i in range(1, samples + 1):
            req = json.dumps({"T": 105}, separators=(",", ":"))
            ser.write(req.encode() + b"\n")
            logger.debug(f"Sent telemetry request ({i}/{samples}): {req}")
            time.sleep(0.15)

            got_response = False
            while ser.in_waiting:
                raw = ser.readline().decode("utf-8", errors="replace").strip()
                if not raw:
                    continue
                got_response = True
                try:
                    data = json.loads(raw)
                except json.JSONDecodeError:
                    logger.warning(f"Non-JSON response: {raw!r}")
                    continue

                voltage = next((data[k] for k in _VOLTAGE_KEYS if k in data), None)
                if voltage is not None:
                    status = _interpret_voltage(float(voltage))
                    logger.success(f"[{i}/{samples}] Battery: {voltage} V — {status}  (full response: {data})")
                else:
                    # Print the whole response so we can see what keys the firmware uses.
                    logger.info(f"[{i}/{samples}] Response (no voltage key detected): {data}")

            if not got_response:
                logger.warning(f"[{i}/{samples}] No response received — check port and baud rate.")

            if i < samples:
                time.sleep(interval)


def main() -> None:
    parser = argparse.ArgumentParser(description="Check Waveshare UGV Rover battery voltage.")
    parser.add_argument(
        "--port",
        default="/dev/ttyAMA0",
        help="Serial port for the UGV sub-controller (default: /dev/ttyAMA0 for Pi 5; "
             "use /dev/serial0 for Pi 4B)",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=5,
        help="Number of telemetry readings to take (default: 5)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.5,
        help="Seconds between readings (default: 0.5)",
    )
    args = parser.parse_args()
    check_battery(args.port, args.samples, args.interval)


if __name__ == "__main__":
    main()
