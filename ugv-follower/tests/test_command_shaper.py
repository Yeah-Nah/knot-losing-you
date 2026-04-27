"""Unit tests for CommandShaper and the _step_wheel pure function.

Module under test
-----------------
ugv_follower.control.command_shaper — Background thread that rate-limits wheel
commands and enforces a reversal dwell around zero crossing.
ugv_follower.control.ugv_controller — Serial transport layer tested with shaping
enabled and disabled.

Test groups
-----------
1  TestStepWheelRamp          — ramp behaviour of the pure ``_step_wheel`` function; no threading.
2  TestStepWheelReversal      — direction-reversal logic in ``_step_wheel``.
3  TestStepWheelDwell         — dwell-on-zero enforcement in ``_step_wheel``.
4  TestCommandShaperThreading — ``CommandShaper`` lifecycle with a mocked send function.
5  TestImmediateStop          — ``immediate_stop()`` clears the queue and sends zero.
6  TestUGVControllerShaping   — ``UGVController`` with shaping enabled/disabled over a patched serial port.

Running
-------
All tests in this file (from ``ugv-follower/``)::

    pytest tests/test_command_shaper.py

Verbose with short tracebacks::

    pytest tests/test_command_shaper.py -v --tb=short

Run a specific group::

    pytest tests/test_command_shaper.py::TestStepWheelRamp
    pytest tests/test_command_shaper.py::TestImmediateStop

Notes
-----
No physical hardware required.  Serial I/O is patched with ``unittest.mock``.
Threading integration tests use real threads with short timeouts; they may be
sensitive to heavily loaded machines.
"""

from __future__ import annotations

import threading
import time
from typing import Callable
from unittest.mock import patch

import pytest

from ugv_follower.control.command_shaper import CommandShaper, _step_wheel
from ugv_follower.control.ugv_controller import UGVController

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_EPSILON = 0.01
_MAX_STEP = 0.1
_DWELL_ON_ZERO = 5


def _step(
    current: float,
    target: float,
    dwell: int = 0,
    max_step: float = _MAX_STEP,
    epsilon: float = _EPSILON,
    dwell_on_zero: int = _DWELL_ON_ZERO,
) -> tuple[float, int]:
    """Thin wrapper around _step_wheel with convenient defaults."""
    new_current, new_dwell = _step_wheel(
        current, target, dwell, max_step, epsilon, dwell_on_zero
    )
    return float(new_current), int(new_dwell)


# ---------------------------------------------------------------------------
# Group 1 — Pure _step_wheel unit tests
# ---------------------------------------------------------------------------


class TestStepWheelRamp:
    """Normal forward ramping (Phase 4, no reversal)."""

    def test_ramps_toward_positive_target(self) -> None:
        """Wheel at zero advances by max_step toward a positive target."""
        new_c, new_d = _step(0.0, 1.0)
        assert new_c == pytest.approx(_MAX_STEP)
        assert new_d == 0

    def test_ramps_toward_negative_target_from_zero(self) -> None:
        """Wheel at zero advances by max_step toward a negative target."""
        new_c, new_d = _step(0.0, -1.0)
        assert new_c == pytest.approx(-_MAX_STEP)
        assert new_d == 0

    def test_does_not_overshoot_target(self) -> None:
        """When remaining gap < max_step, snaps to target exactly."""
        new_c, new_d = _step(0.95, 1.0)
        assert new_c == pytest.approx(1.0)
        assert new_d == 0

    def test_at_target_returns_target(self) -> None:
        """When current == target (within epsilon), returns target with zero dwell."""
        new_c, new_d = _step(1.0, 1.0)
        assert new_c == pytest.approx(1.0)
        assert new_d == 0

    def test_at_target_within_epsilon_snaps(self) -> None:
        """current within epsilon of target is treated as at-target."""
        new_c, new_d = _step(1.005, 1.0, epsilon=0.01)
        assert new_c == pytest.approx(1.0)
        assert new_d == 0


class TestStepWheelReversal:
    """Reversal protection — sign flip causes zero-crossing + dwell."""

    def test_reversal_ramps_toward_zero_not_target(self) -> None:
        """When target flips sign, current moves toward zero, not toward target."""
        new_c, new_d = _step(0.5, -1.0)
        assert new_c == pytest.approx(0.4)
        assert new_d == 0  # dwell not started yet

    def test_reversal_negative_to_positive_ramps_toward_zero(self) -> None:
        """Same logic in the negative-to-positive direction."""
        new_c, new_d = _step(-0.5, 1.0)
        assert new_c == pytest.approx(-0.4)
        assert new_d == 0

    def test_reversal_initiates_dwell_on_zero_crossing(self) -> None:
        """When ramp reaches zero, dwell_ticks_on_zero is returned."""
        # current small enough that one max_step crosses zero
        new_c, new_d = _step(0.05, -1.0)
        assert new_c == pytest.approx(0.0)
        assert new_d == _DWELL_ON_ZERO

    def test_reversal_negative_crossing_initiates_dwell(self) -> None:
        new_c, new_d = _step(-0.05, 1.0)
        assert new_c == pytest.approx(0.0)
        assert new_d == _DWELL_ON_ZERO

    def test_near_zero_current_skips_reversal(self) -> None:
        """abs(current) <= epsilon means motor is effectively stopped — no reversal."""
        new_c, new_d = _step(0.005, -1.0, epsilon=0.01)
        # Falls through to Phase 4: ramps toward -1.0
        assert new_c < 0.0
        assert new_d == 0

    def test_zero_target_does_not_trigger_reversal(self) -> None:
        """target == 0 gives target*current == 0, so reversal branch is skipped."""
        new_c, new_d = _step(0.5, 0.0)
        # Phase 4: decelerates toward 0
        assert new_c == pytest.approx(0.4)
        assert new_d == 0


class TestStepWheelDwell:
    """Dwell phase — hold at zero, then resume ramping."""

    def test_dwell_holds_at_zero_and_decrements(self) -> None:
        """During dwell, current stays 0 and counter decrements."""
        new_c, new_d = _step(0.0, -1.0, dwell=3)
        assert new_c == pytest.approx(0.0)
        assert new_d == 2

    def test_dwell_last_tick_still_holds_zero(self) -> None:
        """When dwell_ticks == 1, this tick holds at zero and returns 0."""
        new_c, new_d = _step(0.0, -1.0, dwell=1)
        assert new_c == pytest.approx(0.0)
        assert new_d == 0

    def test_after_dwell_ramps_toward_target(self) -> None:
        """Once dwell expires (dwell=0), Phase 4 ramps toward target."""
        new_c, new_d = _step(0.0, -1.0, dwell=0)
        assert new_c == pytest.approx(-_MAX_STEP)
        assert new_d == 0

    def test_zero_dwell_on_zero_no_dwell_inserted(self) -> None:
        """dwell_ticks_on_zero == 0 means zero-crossing has no pause."""
        new_c, new_d = _step(0.05, -1.0, dwell_on_zero=0)
        assert new_c == pytest.approx(0.0)
        assert new_d == 0  # no dwell


# ---------------------------------------------------------------------------
# Group 2 — CommandShaper threading integration
# ---------------------------------------------------------------------------


class TestCommandShaperThreading:
    """Integration tests with a mock send_fn and a live background thread."""

    def _make_shaper(
        self,
        send_fn: Callable[[float, float], None],
        update_rate_hz: float = 200.0,
        ramp_rate_per_s: float = 4.0,
        reversal_dwell_s: float = 0.0,
    ) -> CommandShaper:
        return CommandShaper(
            send_fn=send_fn,
            update_rate_hz=update_rate_hz,
            ramp_rate_per_s=ramp_rate_per_s,
            reversal_dwell_s=reversal_dwell_s,
            zero_crossing_epsilon=0.01,
        )

    def test_calls_send_fn_repeatedly(self) -> None:
        """The thread fires send_fn multiple times per 150 ms window."""
        calls: list[tuple[float, float]] = []
        lock = threading.Lock()

        def mock_send(left: float, right: float) -> None:
            with lock:
                calls.append((left, right))

        shaper = self._make_shaper(mock_send, update_rate_hz=100.0)
        shaper.start()
        time.sleep(0.15)
        shaper.stop()

        with lock:
            n = len(calls)
        assert n >= 10, f"Expected >= 10 send calls in 150 ms, got {n}"

    def test_ramps_to_target(self) -> None:
        """After set_target, shaper converges to the target speed."""
        outputs: list[tuple[float, float]] = []
        lock = threading.Lock()

        def mock_send(left: float, right: float) -> None:
            with lock:
                outputs.append((left, right))

        shaper = self._make_shaper(mock_send)
        shaper.start()
        shaper.set_target(1.0, 1.0)
        time.sleep(0.4)  # allow full ramp at 4 m/s²
        shaper.stop()

        with lock:
            last_l, last_r = outputs[-1]
        assert last_l == pytest.approx(1.0, abs=0.02)
        assert last_r == pytest.approx(1.0, abs=0.02)

    def test_reversal_passes_through_zero(self) -> None:
        """No tick skips zero during a sign reversal (no direct sign jump)."""
        outputs: list[tuple[float, float]] = []
        lock = threading.Lock()

        def mock_send(left: float, right: float) -> None:
            with lock:
                outputs.append((left, right))

        shaper = self._make_shaper(mock_send, ramp_rate_per_s=4.0)
        shaper.start()
        shaper.set_target(1.0, 1.0)
        time.sleep(0.35)  # ramp up to ~1.0

        shaper.set_target(-1.0, -1.0)
        time.sleep(0.7)  # ramp down, cross zero, ramp to -1
        shaper.stop()

        with lock:
            all_lefts = [left for left, _ in outputs]

        for i in range(1, len(all_lefts)):
            prev, curr = all_lefts[i - 1], all_lefts[i]
            if prev > 0.05 and curr < -0.05:
                pytest.fail(f"Direct sign flip at index {i}: {prev:.4f} → {curr:.4f}")

    def test_already_running_raises(self) -> None:
        """Starting an already-running shaper raises RuntimeError."""
        shaper = CommandShaper(send_fn=lambda left, right: None)
        shaper.start()
        try:
            with pytest.raises(RuntimeError, match="already running"):
                shaper.start()
        finally:
            shaper.stop()

    def test_stop_before_start_is_safe(self) -> None:
        """Calling stop() without ever calling start() does not raise."""
        shaper = CommandShaper(send_fn=lambda left, right: None)
        shaper.stop()  # must not raise


# ---------------------------------------------------------------------------
# Group 3 — immediate_stop() behaviour
# ---------------------------------------------------------------------------


class TestImmediateStop:
    """immediate_stop() must zero state and call send_fn synchronously."""

    def test_sends_zero_synchronously(self) -> None:
        """After immediate_stop(), the last recorded send is (0.0, 0.0)."""
        sent: list[tuple[float, float]] = []

        def mock_send(left: float, right: float) -> None:
            sent.append((left, right))

        shaper = CommandShaper(
            send_fn=mock_send,
            update_rate_hz=10.0,
            ramp_rate_per_s=1.0,
            reversal_dwell_s=0.05,
            zero_crossing_epsilon=0.01,
        )
        shaper.start()
        shaper.set_target(1.0, 1.0)
        time.sleep(0.05)

        shaper.immediate_stop()
        # The synchronous call inside immediate_stop must have fired immediately.
        assert sent[-1] == (0.0, 0.0)

        shaper.stop()

    def test_resets_all_state_to_zero(self) -> None:
        """After immediate_stop(), the shaper keeps sending (0, 0), not the prior target."""
        outputs: list[tuple[float, float]] = []
        lock = threading.Lock()

        def mock_send(left: float, right: float) -> None:
            with lock:
                outputs.append((left, right))

        shaper = CommandShaper(
            send_fn=mock_send,
            update_rate_hz=100.0,
            ramp_rate_per_s=5.0,
            reversal_dwell_s=0.0,
            zero_crossing_epsilon=0.01,
        )
        shaper.start()
        shaper.set_target(1.0, 1.0)
        time.sleep(0.1)
        shaper.immediate_stop()
        time.sleep(0.1)  # let a few post-stop ticks fire
        shaper.stop()

        with lock:
            # Find the last non-zero output; everything after must be (0, 0).
            last_nonzero = -1
            for i, (left, right) in enumerate(outputs):
                if abs(left) > 0.02 or abs(right) > 0.02:
                    last_nonzero = i
            for left, right in outputs[last_nonzero + 1 :]:
                assert abs(left) < 0.02
                assert abs(right) < 0.02


# ---------------------------------------------------------------------------
# Group 4 — UGVController integration (patched serial)
# ---------------------------------------------------------------------------


class TestUGVControllerShaping:
    """UGVController shaping on/off behaviour without real hardware."""

    def _make_controller(
        self, shaping_enabled: bool = False, update_rate_hz: float = 50.0
    ) -> UGVController:
        return UGVController(
            port="/dev/null",
            shaping_enabled=shaping_enabled,
            update_rate_hz=update_rate_hz,
        )

    def test_shaping_disabled_move_sends_immediately(self) -> None:
        """With shaping off, move() calls _send_wheel_speeds synchronously."""
        with patch("ugv_follower.control.ugv_controller.serial.Serial") as mock_serial:
            mock_serial.return_value.is_open = True
            ctrl = self._make_controller(shaping_enabled=False)
            ctrl._serial = mock_serial.return_value

            captured: list[dict[str, object]] = []

            def capturing_send(cmd: dict[str, object]) -> None:
                captured.append(cmd)

            with patch.object(ctrl, "_send", side_effect=capturing_send):
                ctrl.move(linear=0.5, angular=0.0)

        assert len(captured) == 1
        assert captured[0]["T"] == 1
        assert captured[0]["L"] == pytest.approx(0.5)
        assert captured[0]["R"] == pytest.approx(0.5)

    def test_shaping_enabled_move_is_nonblocking(self) -> None:
        """With shaping on, move() updates the shaper target and writes nothing to serial."""
        with patch("ugv_follower.control.ugv_controller.serial.Serial") as mock_serial:
            mock_serial.return_value.is_open = True
            ctrl = self._make_controller(shaping_enabled=True, update_rate_hz=10.0)
            ctrl._serial = mock_serial.return_value

            write_calls: list[bytes] = []
            mock_serial.return_value.write.side_effect = write_calls.append

            assert ctrl._shaper is not None

            ctrl.move(linear=0.5, angular=0.0)

            # move() must not have written anything to serial.
            assert len(write_calls) == 0

            # Shaper target must reflect the commanded wheel speeds.
            half = ctrl._track_width / 2.0
            expected_l = 0.5 - 0.0 * half
            expected_r = 0.5 + 0.0 * half
            with ctrl._shaper._lock:
                assert ctrl._shaper._left_target == pytest.approx(expected_l)
                assert ctrl._shaper._right_target == pytest.approx(expected_r)

    def test_shaping_enabled_stop_calls_immediate_stop(self) -> None:
        """stop() with shaping on delegates to shaper.immediate_stop()."""
        with patch("ugv_follower.control.ugv_controller.serial.Serial") as mock_serial:
            mock_serial.return_value.is_open = True
            ctrl = self._make_controller(shaping_enabled=True)
            ctrl._serial = mock_serial.return_value

            assert ctrl._shaper is not None
            with patch.object(ctrl._shaper, "immediate_stop") as immediate_stop:
                ctrl.stop()
                immediate_stop.assert_called_once()

    def test_shaping_disabled_stop_sends_zero_directly(self) -> None:
        """stop() with shaping off writes {T:1, L:0, R:0} directly."""
        with patch("ugv_follower.control.ugv_controller.serial.Serial") as mock_serial:
            mock_serial.return_value.is_open = True
            ctrl = self._make_controller(shaping_enabled=False)
            ctrl._serial = mock_serial.return_value

            captured: list[dict[str, object]] = []
            with patch.object(ctrl, "_send", side_effect=captured.append):
                ctrl.stop()

        assert any(
            cmd.get("T") == 1 and cmd.get("L") == 0 and cmd.get("R") == 0
            for cmd in captured
        )

    def test_shaper_none_when_disabled(self) -> None:
        """No CommandShaper is created when shaping_enabled=False."""
        ctrl = self._make_controller(shaping_enabled=False)
        assert ctrl._shaper is None

    def test_shaper_created_when_enabled(self) -> None:
        """A CommandShaper is created when shaping_enabled=True."""
        ctrl = self._make_controller(shaping_enabled=True)
        assert ctrl._shaper is not None
        assert isinstance(ctrl._shaper, CommandShaper)
