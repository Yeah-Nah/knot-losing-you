"""Motion command contract for Phase 3A-lite.

Defines one canonical payload shape for chassis motion commands so all
upstream decision code and downstream control code speak the same interface.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import StrEnum
from typing import Protocol


class MotionController(Protocol):
    """Minimal controller interface required by apply_motion_command."""

    def move(self, linear: float, angular: float) -> None: ...


class MotionCommandSource(StrEnum):
    """Origin of a motion command."""

    AUTONOMOUS = "autonomous"
    MANUAL = "manual"
    ESTOP = "estop"


@dataclass(frozen=True)
class MotionCommand:
    """Normalized chassis motion command.

    Parameters
    ----------
    linear_m_s : float
        Forward speed in metres per second. Positive is forward.
    angular_rad_s : float
        Yaw rate in radians per second. Positive is counter-clockwise.
    source : MotionCommandSource
        Origin of this command.
    timestamp_s : float
        Monotonic timestamp in seconds when this command was produced.
    reason : str
        Short runtime reason tag describing why this command was emitted.
    """

    linear_m_s: float
    angular_rad_s: float
    source: MotionCommandSource
    timestamp_s: float
    reason: str = ""

    @classmethod
    def zero(
        cls,
        source: MotionCommandSource,
        reason: str = "",
    ) -> MotionCommand:
        """Build a zero-velocity command stamped with monotonic time."""
        return cls(
            linear_m_s=0.0,
            angular_rad_s=0.0,
            source=source,
            timestamp_s=time.monotonic(),
            reason=reason,
        )

    def validate(self) -> None:
        """Validate numeric fields and timestamp semantics.

        Raises
        ------
        ValueError
            If any numeric field is not finite or timestamp is non-positive.
        """
        if not math.isfinite(self.linear_m_s):
            raise ValueError("linear_m_s must be finite")
        if not math.isfinite(self.angular_rad_s):
            raise ValueError("angular_rad_s must be finite")
        if not math.isfinite(self.timestamp_s) or self.timestamp_s <= 0.0:
            raise ValueError("timestamp_s must be finite and > 0")


def apply_motion_command(controller: MotionController, command: MotionCommand) -> None:
    """Single downstream adapter from normalized command to controller API."""
    command.validate()
    controller.move(linear=command.linear_m_s, angular=command.angular_rad_s)
