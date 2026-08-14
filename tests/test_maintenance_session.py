"""机械臂维护独占会话的公共运行时测试。"""

from __future__ import annotations

import time

import pytest
from unilab_robot_contracts import (
    ActionKind,
    CommandResult,
    CommandState,
    CommissioningCapabilities,
    CommissioningSnapshot,
    DeploymentMode,
    MotionSegment,
    MoveTargetCommand,
    ObservationState,
    RobotCommand,
)
from unilab_robot_runtime import bind_commissioning_runtime, build_test_runtime


class FakeRuntime:
    """记录生产命令的最小 standalone 运行时。"""

    def __init__(self) -> None:
        self.commands: list[str] = []

    def execute(self, command: RobotCommand) -> CommandResult:
        self.commands.append(command.command_id)
        return CommandResult(command.command_id, CommandState.SUCCEEDED, "done")


class FakeCommissioningPort:
    """返回新鲜空闲状态并记录维护命令。"""

    def __init__(self) -> None:
        self.commands: list[str] = []

    @property
    def commissioning_capabilities(self) -> CommissioningCapabilities:
        return CommissioningCapabilities(True, True, True, True, True)

    @property
    def commissioning_target_revision(self) -> str:
        return "points@1.0.0"

    def commissioning_snapshot(self) -> CommissioningSnapshot:
        return CommissioningSnapshot(
            ObservationState.KNOWN,
            time.time(),
            1.0,
            "test:maintenance",
            True,
            True,
            None,
            False,
        )

    def execute_commissioning(self, command: MoveTargetCommand) -> CommandResult:
        self.commands.append(command.command_id)
        return CommandResult(command.command_id, CommandState.SUCCEEDED, "done")


def _production_command(command_id: str) -> RobotCommand:
    return RobotCommand(
        command_id,
        ActionKind.PICK,
        "profile-digest",
        "beaker",
        "boot-1",
        1,
        (MotionSegment("approach", "position1.approach"),),
    )


def _maintenance_command() -> MoveTargetCommand:
    return MoveTargetCommand(
        command_id="maintenance-1",
        hardware_profile_digest="profile-digest",
        source_boot_id="boot-1",
        monotonic_sequence=1,
        motion_profile_ref="maintenance-slow",
        velocity_scale=0.05,
        acceleration_scale=0.05,
        target_ref="position1.approach",
        target_revision="points@1.0.0",
    )


def test_maintenance_session_excludes_production_until_closed() -> None:
    """同一物理端点在维护会话期间不得接受生产 RobotCommand。"""

    runtime = FakeRuntime()
    commissioning = FakeCommissioningPort()
    binding = build_test_runtime(
        runtime,
        frozenset({"moveit:test"}),
        owner_id="runtime-test",
        commissioning_port=commissioning,
        deployment_mode=DeploymentMode.SIMULATION,
    )

    session = binding.open_maintenance_session("operator-a")
    try:
        assert session.execute(_maintenance_command()).success
        with pytest.raises(RuntimeError, match="维护会话"):
            binding.execute(_production_command("production-blocked"))
        with pytest.raises(RuntimeError, match="已被占用"):
            binding.open_maintenance_session("operator-b")
    finally:
        session.close()

    try:
        assert binding.execute(_production_command("production-after")).success
    finally:
        binding.close()


def test_commissioning_only_binding_exposes_maintenance_without_production() -> None:
    """领域包可暴露既有 Adapter，但不能因此创建第二个生产执行入口。"""

    commissioning = FakeCommissioningPort()
    binding = bind_commissioning_runtime(
        commissioning,
        frozenset({"moveit:robot:cr5"}),
        owner_id="robot-commissioning",
        deployment_mode=DeploymentMode.SIMULATION,
    )

    session = binding.open_maintenance_session("workbench:operator-a")
    try:
        assert session.execute(_maintenance_command()).success
        with pytest.raises(RuntimeError, match="维护会话"):
            binding.execute(_production_command("must-not-dispatch"))
    finally:
        session.close()

    try:
        with pytest.raises(RuntimeError, match="不提供生产动作入口"):
            binding.execute(_production_command("must-not-dispatch-after-close"))
    finally:
        binding.close()
