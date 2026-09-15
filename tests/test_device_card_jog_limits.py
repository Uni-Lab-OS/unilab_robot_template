"""机械臂卡片 Jog 步长上限（template 通用层）。"""

from __future__ import annotations

import math

import pytest

from unilab_robot_runtime.device_card.context import ArmCardContext
from unilab_robot_runtime.device_card import manual_motion


class _CommissioningPort:
    hardware_profile_digest = "test-hardware-profile"
    commissioning_velocity_limit = 0.1
    commissioning_acceleration_limit = 0.1
    commissioning_motion_profile_ref = "szlab:test-jog"


class _Binding:
    commissioning_port = _CommissioningPort()


_CONTEXT = ArmCardContext(boot_id_prefix="szlab")


def _capture_commands(monkeypatch: pytest.MonkeyPatch) -> list[object]:
    commands: list[object] = []

    def capture(_binding: object, command: object, *, owner: str) -> dict[str, object]:
        commands.append(command)
        return {"success": True, "message": owner, "command_id": command.command_id}

    monkeypatch.setattr(manual_motion, "_execute", capture)
    return commands


def test_joint_jog_accepts_step_above_previous_card_maximum(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    commands = _capture_commands(monkeypatch)

    manual_motion.jog_joint_once(
        _Binding(),
        _CONTEXT,
        device_id="szlab_mixer_robot",
        monotonic_sequence=1,
        joint_ref="joint_6",
        direction="positive",
        step_deg=360.0,
    )

    assert len(commands) == 1
    assert math.isclose(commands[0].step_si, math.radians(360.0))


def test_tcp_jog_accepts_translation_and_rotation_above_previous_maximum(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    commands = _capture_commands(monkeypatch)

    manual_motion.jog_tcp_once(
        _Binding(),
        _CONTEXT,
        device_id="szlab_mixer_robot",
        monotonic_sequence=1,
        axis="x",
        direction="positive",
        step=2_000.0,
        frame_ref="arm_base",
    )
    manual_motion.jog_tcp_once(
        _Binding(),
        _CONTEXT,
        device_id="szlab_mixer_robot",
        monotonic_sequence=2,
        axis="rz",
        direction="negative",
        step=720.0,
        frame_ref="tool",
    )

    assert len(commands) == 2
    assert math.isclose(commands[0].step_si, 2.0)
    assert math.isclose(commands[1].step_si, math.radians(720.0))
