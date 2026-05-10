"""Unit tests for UGVController.query_pan_deg serial query behaviour.

Module under test
-----------------
ugv_follower.control.ugv_controller — query_pan_deg RX-flush and parse logic.

Test groups
-----------
1  TestQueryPanDeg — flush-before-write, valid response, closed port, timeout.

Running
-------
All tests in this file (from ``ugv-follower/``)::

    pytest tests/test_ugv_controller.py

Verbose with short tracebacks::

    pytest tests/test_ugv_controller.py -v --tb=short
"""

from __future__ import annotations

import json
from unittest.mock import MagicMock, call, patch

import pytest

from ugv_follower.control.ugv_controller import UGVController


def _make_controller() -> UGVController:
    return UGVController(port="/dev/null", shaping_enabled=False)


class TestQueryPanDeg:
    """query_pan_deg flush-before-write and parse behaviour."""

    def test_query_pan_deg_flushes_before_write(self) -> None:
        """reset_input_buffer() must be called before write() inside the lock."""
        with patch("ugv_follower.control.ugv_controller.serial.Serial") as mock_serial_cls:
            mock_port = MagicMock()
            mock_port.is_open = True
            mock_serial_cls.return_value = mock_port
            mock_port.readline.return_value = b""

            ctrl = _make_controller()
            ctrl._serial = mock_port

            ctrl.query_pan_deg(timeout_s=0.0)

        # Collect the subset of calls we care about.
        call_names = [c[0] for c in mock_port.method_calls]
        assert "reset_input_buffer" in call_names
        assert "write" in call_names
        flush_idx = call_names.index("reset_input_buffer")
        write_idx = call_names.index("write")
        assert flush_idx < write_idx, (
            f"reset_input_buffer (pos {flush_idx}) must precede write (pos {write_idx})"
        )

    def test_query_pan_deg_returns_pan_on_valid_response(self) -> None:
        """Returns the float pan value from a well-formed T=1001 response."""
        response = json.dumps({"T": 1001, "pan": 12.5}).encode() + b"\n"

        with patch("ugv_follower.control.ugv_controller.serial.Serial") as mock_serial_cls:
            mock_port = MagicMock()
            mock_port.is_open = True
            mock_serial_cls.return_value = mock_port
            mock_port.readline.return_value = response

            ctrl = _make_controller()
            ctrl._serial = mock_port

            result = ctrl.query_pan_deg(timeout_s=0.1)

        assert result == pytest.approx(12.5)

    def test_query_pan_deg_returns_none_when_port_closed(self) -> None:
        """Returns None immediately when _serial is None."""
        ctrl = _make_controller()
        ctrl._serial = None

        assert ctrl.query_pan_deg() is None

    def test_query_pan_deg_returns_none_on_timeout(self) -> None:
        """Returns None when readline yields no data within the timeout."""
        with patch("ugv_follower.control.ugv_controller.serial.Serial") as mock_serial_cls:
            mock_port = MagicMock()
            mock_port.is_open = True
            mock_serial_cls.return_value = mock_port
            mock_port.readline.return_value = b""

            ctrl = _make_controller()
            ctrl._serial = mock_port

            result = ctrl.query_pan_deg(timeout_s=0.0)

        assert result is None
