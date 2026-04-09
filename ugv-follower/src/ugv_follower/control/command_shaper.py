"""Background wheel-command shaper for the Waveshare UGV Rover.

Applies two protections to wheel speed commands before they reach hardware:

1. **Reversal cool-down**: when a wheel is still carrying significant speed in
   one direction and the target flips sign, the shaper ramps the wheel to zero
   and holds it there for a configurable dwell period before allowing it to
   accelerate in the opposite direction.  This prevents abrupt sign-flipped
   motor commands that stress the drivetrain.

2. **Rate limiting**: limits how quickly the output speed can change per tick,
   smoothing sudden large step changes in the commanded velocity.

The shaper runs in a daemon thread and calls a caller-supplied ``send_fn``
callback at ``update_rate_hz`` frequency.  :meth:`CommandShaper.set_target`
is thread-safe and returns immediately, so callers (e.g. a real-time control
loop) are never blocked.

:func:`_step_wheel` is a pure module-level function so it can be unit-tested
independently of the threading machinery.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Callable

from loguru import logger


def _step_wheel(
    current: float,
    target: float,
    dwell_ticks: int,
    max_step: float,
    epsilon: float,
    dwell_ticks_on_zero: int,
) -> tuple[float, int]:
    """Advance one wheel's output speed by one shaper tick.

    Parameters
    ----------
    current : float
        Current wheel output speed in m/s.  Negative = reverse.
    target : float
        Desired wheel output speed in m/s.
    dwell_ticks : int
        Remaining ticks to hold the wheel at zero after a sign crossing.
        Must be >= 0.
    max_step : float
        Maximum absolute speed change permitted in one tick (m/s).
        Equals ``ramp_rate_per_s / update_rate_hz``.
    epsilon : float
        Speeds with ``|v| <= epsilon`` are treated as zero for sign-change
        detection (``zero_crossing_epsilon`` in the shaper config).
    dwell_ticks_on_zero : int
        Number of ticks to dwell at zero when a reversal zero-crossing occurs.
        Equals ``round(reversal_dwell_s * update_rate_hz)``.

    Returns
    -------
    tuple[float, int]
        ``(new_current, new_dwell_ticks)`` — updated wheel speed and remaining
        dwell ticks after this tick.

    Notes
    -----
    State machine (evaluated in order):

    1. **At target** — ``abs(current - target) <= epsilon``: snap to target,
       clear dwell.
    2. **Reversal pending** — ``target * current < 0`` and
       ``abs(current) > epsilon``: ramp *current* toward zero.  When it
       crosses (``abs(new) <= epsilon``), set ``new = 0`` and start dwell.
    3. **Dwell** — ``dwell_ticks > 0``: hold at zero, decrement counter.
    4. **Normal ramp** — step *current* toward *target* by at most
       *max_step*.

    A zero *target* never triggers reversal (``target * current == 0``); the
    wheel simply decelerates via state 4.  The hardware ``stop()`` bypasses
    the shaper entirely so this edge case is safe.
    """
    # State 1 — already at target.
    if abs(current - target) <= epsilon:
        return target, 0

    # State 2 — reversal: target sign differs from current and motor is live.
    if target * current < 0.0 and abs(current) > epsilon:
        step = min(max_step, abs(current))
        new_current = current - math.copysign(step, current)
        if abs(new_current) <= epsilon:
            return 0.0, dwell_ticks_on_zero
        return new_current, 0

    # State 3 — dwell at zero after a zero-crossing.
    if dwell_ticks > 0:
        return 0.0, dwell_ticks - 1

    # State 4 — normal ramp toward target.
    delta = target - current
    step = min(max_step, abs(delta))
    return current + math.copysign(step, delta), 0


class CommandShaper:
    """Background-threaded wheel command shaper.

    Sits between :meth:`UGVController.move` and the serial send function.
    :meth:`set_target` accepts new wheel speed targets and returns
    immediately; the background thread applies :func:`_step_wheel` to both
    wheels at ``update_rate_hz`` and calls ``send_fn`` with the shaped
    outputs.

    Parameters
    ----------
    send_fn : Callable[[float, float], None]
        Called with ``(left_speed, right_speed)`` on every tick.  Must be
        safe to call from a background thread.
    update_rate_hz : float
        Tick rate of the shaper loop (Hz).  50 Hz is fine at 115200 baud.
    ramp_rate_per_s : float
        Maximum wheel speed change per second (m/s per s).  A value of 2.0
        means the wheel reaches 1 m/s in 0.5 s from rest.
    reversal_dwell_s : float
        Time (seconds) to hold a wheel at zero after a zero-crossing before
        allowing acceleration in the opposite direction.
    zero_crossing_epsilon : float
        Wheel speeds with ``|v| <= epsilon`` are treated as zero for
        sign-change detection.
    """

    def __init__(
        self,
        send_fn: Callable[[float, float], None],
        update_rate_hz: float = 50.0,
        ramp_rate_per_s: float = 2.0,
        reversal_dwell_s: float = 0.05,
        zero_crossing_epsilon: float = 0.01,
    ) -> None:
        self._send_fn = send_fn
        self._dt: float = 1.0 / update_rate_hz
        self._max_step: float = ramp_rate_per_s / update_rate_hz
        self._dwell_ticks: int = round(reversal_dwell_s * update_rate_hz)
        self._epsilon: float = zero_crossing_epsilon

        self._lock = threading.Lock()
        self._left_current: float = 0.0
        self._right_current: float = 0.0
        self._left_target: float = 0.0
        self._right_target: float = 0.0
        self._left_dwell: int = 0
        self._right_dwell: int = 0

        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start the background shaper thread.

        Raises
        ------
        RuntimeError
            If the shaper is already running.
        """
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("CommandShaper is already running.")
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run_loop,
            name="CommandShaperThread",
            daemon=True,
        )
        self._thread.start()
        logger.debug("CommandShaper started.")

    def stop(self) -> None:
        """Signal the shaper thread to exit and wait for it to join.

        Safe to call even if the shaper was never started.
        """
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                logger.warning("CommandShaper thread did not exit within 2 s timeout.")
            self._thread = None
        logger.debug("CommandShaper stopped.")

    def set_target(self, left: float, right: float) -> None:
        """Update the target wheel speeds.

        Thread-safe.  Returns immediately — the shaper loop picks up the new
        targets on its next tick.

        Parameters
        ----------
        left : float
            Target left wheel speed in m/s.
        right : float
            Target right wheel speed in m/s.
        """
        with self._lock:
            self._left_target = left
            self._right_target = right

    def immediate_stop(self) -> None:
        """Zero all shaper state and send ``(0, 0)`` to hardware immediately.

        Bypasses the shaper loop — safe for emergencies and ``disconnect()``.
        Resets current speeds, targets, and dwell counters so the next
        :meth:`set_target` call starts from a clean zero state.

        Thread-safe.
        """
        with self._lock:
            self._left_current = 0.0
            self._right_current = 0.0
            self._left_target = 0.0
            self._right_target = 0.0
            self._left_dwell = 0
            self._right_dwell = 0
        # Call send_fn outside the lock — serial I/O must not hold _lock.
        # Worst-case race: the thread reads freshly-zeroed state and sends
        # another (0, 0) on its next tick — benign.
        self._send_fn(0.0, 0.0)
        logger.debug("CommandShaper: immediate stop applied.")

    # ------------------------------------------------------------------
    # Private — thread body
    # ------------------------------------------------------------------

    def _run_loop(self) -> None:
        """Shaper tick loop.  Runs at ``update_rate_hz`` until stopped."""
        while not self._stop_event.is_set():
            tick_start = time.monotonic()

            # Read current state under lock (brief critical section).
            with self._lock:
                lc = self._left_current
                lt = self._left_target
                ld = self._left_dwell
                rc = self._right_current
                rt = self._right_target
                rd = self._right_dwell

            new_lc, new_ld = _step_wheel(
                lc, lt, ld, self._max_step, self._epsilon, self._dwell_ticks
            )
            new_rc, new_rd = _step_wheel(
                rc, rt, rd, self._max_step, self._epsilon, self._dwell_ticks
            )

            # Write shaped outputs under lock (brief critical section).
            with self._lock:
                self._left_current = new_lc
                self._right_current = new_rc
                self._left_dwell = new_ld
                self._right_dwell = new_rd

            # Send outside the lock so set_target / immediate_stop are never
            # blocked behind serial I/O.
            try:
                self._send_fn(new_lc, new_rc)
            except Exception:
                logger.exception("CommandShaper: send_fn raised; continuing.")

            elapsed = time.monotonic() - tick_start
            sleep_s = self._dt - elapsed
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            elif sleep_s < -self._dt:
                logger.warning(
                    f"CommandShaper tick overran by {-sleep_s * 1000:.1f} ms."
                )
