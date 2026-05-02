"""Unit tests for the battery check tool.

Module under test
-----------------
tools.check_battery

Running
-------
From ``ugv-follower/``::

    pytest tests/test_check_battery.py
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch

from tools.check_battery import listen


@patch("tools.check_battery.serial.Serial")
@patch("tools.check_battery.ensure_character_device_available")
def test_listen_runs_serial_preflight_before_open(
    mock_preflight: MagicMock,
    mock_serial_cls: MagicMock,
) -> None:
    """listen() checks port ownership before opening the UART."""
    mock_serial = MagicMock()
    mock_serial.__enter__.return_value = mock_serial
    mock_serial.__exit__.return_value = None
    mock_serial.readline.return_value = b""
    mock_serial.in_waiting = 0
    mock_serial_cls.return_value = mock_serial

    listen("/dev/ttyAMA0", 0.0)

    mock_preflight.assert_called_once_with("/dev/ttyAMA0", device_label="Serial port")
    mock_serial_cls.assert_called_once_with("/dev/ttyAMA0", 115200, timeout=1)


@patch("tools.check_battery.logger")
@patch("tools.check_battery.serial.Serial")
@patch("tools.check_battery.ensure_character_device_available")
def test_listen_returns_when_serial_preflight_fails(
    mock_preflight: MagicMock,
    mock_serial_cls: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """listen() aborts cleanly when another process owns the serial port."""
    mock_preflight.side_effect = RuntimeError("Serial port /dev/ttyAMA0 is busy")

    listen("/dev/ttyAMA0", 0.0)

    mock_serial_cls.assert_not_called()
    mock_logger.error.assert_called_once()
