"""Unit tests for the Phase 3A motion command contract."""

from __future__ import annotations

import math
import time

import pytest

from ugv_follower.control.motion_command import (
    MotionCommand,
    MotionCommandSource,
    apply_motion_command,
)


class _DummyController:
    def __init__(self) -> None:
        self.calls: list[tuple[float, float]] = []

    def move(self, linear: float, angular: float) -> None:
        self.calls.append((linear, angular))


def test_zero_factory_uses_normalized_shape() -> None:
    cmd = MotionCommand.zero(source=MotionCommandSource.AUTONOMOUS)

    assert cmd.linear_m_s == 0.0
    assert cmd.angular_rad_s == 0.0
    assert cmd.source == MotionCommandSource.AUTONOMOUS
    assert cmd.timestamp_s > 0.0


def test_validate_accepts_finite_values() -> None:
    cmd = MotionCommand(
        linear_m_s=0.25,
        angular_rad_s=-0.2,
        source=MotionCommandSource.MANUAL,
        timestamp_s=time.monotonic(),
    )

    cmd.validate()  # must not raise


@pytest.mark.parametrize("bad_linear", [math.inf, -math.inf, math.nan])
def test_validate_rejects_non_finite_linear(bad_linear: float) -> None:
    cmd = MotionCommand(
        linear_m_s=bad_linear,
        angular_rad_s=0.1,
        source=MotionCommandSource.AUTONOMOUS,
        timestamp_s=time.monotonic(),
    )

    with pytest.raises(ValueError, match="linear_m_s"):
        cmd.validate()


@pytest.mark.parametrize("bad_angular", [math.inf, -math.inf, math.nan])
def test_validate_rejects_non_finite_angular(bad_angular: float) -> None:
    cmd = MotionCommand(
        linear_m_s=0.1,
        angular_rad_s=bad_angular,
        source=MotionCommandSource.AUTONOMOUS,
        timestamp_s=time.monotonic(),
    )

    with pytest.raises(ValueError, match="angular_rad_s"):
        cmd.validate()


@pytest.mark.parametrize("bad_timestamp", [0.0, -1.0, math.inf, -math.inf, math.nan])
def test_validate_rejects_invalid_timestamp(bad_timestamp: float) -> None:
    cmd = MotionCommand(
        linear_m_s=0.1,
        angular_rad_s=0.2,
        source=MotionCommandSource.ESTOP,
        timestamp_s=bad_timestamp,
    )

    with pytest.raises(ValueError, match="timestamp_s"):
        cmd.validate()


def test_apply_motion_command_validates_and_calls_controller() -> None:
    controller = _DummyController()
    cmd = MotionCommand(
        linear_m_s=0.3,
        angular_rad_s=-0.4,
        source=MotionCommandSource.AUTONOMOUS,
        timestamp_s=time.monotonic(),
    )

    apply_motion_command(controller=controller, command=cmd)

    assert controller.calls == [(0.3, -0.4)]


def test_apply_motion_command_rejects_invalid_command() -> None:
    controller = _DummyController()
    bad_cmd = MotionCommand(
        linear_m_s=math.nan,
        angular_rad_s=0.1,
        source=MotionCommandSource.MANUAL,
        timestamp_s=time.monotonic(),
    )

    with pytest.raises(ValueError):
        apply_motion_command(controller=controller, command=bad_cmd)

    assert controller.calls == []


def test_decide_command_returns_valid_motion_command() -> None:
    from ugv_follower.pipeline import Pipeline, PipelineMode

    pipeline = object.__new__(Pipeline)  # skip __init__, no hardware needed
    pipeline._mode = PipelineMode.AUTONOMOUS
    cmd = pipeline._decide_command()

    assert isinstance(cmd, MotionCommand)
    cmd.validate()  # must be valid by construction


def test_apply_estop_override_passthrough_when_inactive() -> None:
    from ugv_follower.pipeline import Pipeline

    pipeline = object.__new__(Pipeline)  # skip __init__, no hardware needed
    pipeline._estop_active = False
    cmd = MotionCommand(
        linear_m_s=0.2,
        angular_rad_s=0.1,
        source=MotionCommandSource.AUTONOMOUS,
        timestamp_s=time.monotonic(),
    )

    out = pipeline._apply_estop_override(cmd)
    assert out == cmd


def test_apply_estop_override_forces_zero_when_active() -> None:
    from ugv_follower.pipeline import Pipeline

    pipeline = object.__new__(Pipeline)  # skip __init__, no hardware needed
    pipeline._estop_active = True
    cmd = MotionCommand(
        linear_m_s=0.2,
        angular_rad_s=0.1,
        source=MotionCommandSource.AUTONOMOUS,
        timestamp_s=time.monotonic(),
    )

    out = pipeline._apply_estop_override(cmd)
    assert out.linear_m_s == 0.0
    assert out.angular_rad_s == 0.0
    assert out.source == MotionCommandSource.ESTOP


def test_decide_command_reflects_manual_mode() -> None:
    from ugv_follower.pipeline import Pipeline, PipelineMode

    pipeline = object.__new__(Pipeline)  # skip __init__, no hardware needed
    pipeline._mode = PipelineMode.MANUAL
    cmd = pipeline._decide_command()

    assert cmd.source == MotionCommandSource.MANUAL


def test_mode_transition_forces_single_zero_command() -> None:
    from ugv_follower.pipeline import Pipeline, PipelineMode

    pipeline = object.__new__(Pipeline)  # skip __init__, no hardware needed
    pipeline._mode = PipelineMode.AUTONOMOUS
    pipeline._mode_transition_stop_pending = False
    cmd = MotionCommand(
        linear_m_s=0.4,
        angular_rad_s=0.2,
        source=MotionCommandSource.MANUAL,
        timestamp_s=time.monotonic(),
    )

    pipeline.set_mode(PipelineMode.MANUAL)
    first = pipeline._apply_mode_transition_stop(cmd)
    second = pipeline._apply_mode_transition_stop(cmd)

    assert first.linear_m_s == 0.0
    assert first.angular_rad_s == 0.0
    assert first.source == MotionCommandSource.MANUAL
    assert second == cmd
