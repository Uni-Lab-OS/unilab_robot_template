"""OS 权威外部 Fence 接入导轨机械臂协调器的行为测试。"""

from __future__ import annotations

import time
from typing import Any

from unilab_rail_mounted_arm import RailMountedArmCoordinator
from unilab_robot_contracts import (
    ActionKind,
    BackendKind,
    CommandResult,
    CommandState,
    DeploymentMode,
    HardwareProfile,
    InMemoryCommandJournal,
    InterlockMode,
    MotionSegment,
    ObservationState,
    RailStateObservation,
    RobotCommand,
    SafetyInterlockObservation,
)

_UNSET = object()


def _command(command_id: str) -> RobotCommand:
    """构造只携带稳定资产引用的测试命令。"""

    return RobotCommand(
        command_id=command_id,
        action=ActionKind.PICK,
        hardware_profile_digest="profile-digest",
        payload_profile="beaker_500ml@v1",
        source_boot_id="os-fence-test",
        monotonic_sequence=1,
        segments=(MotionSegment("approach", "S04.pick.approach"),),
    )


def _profile() -> HardwareProfile:
    """构造组合端点被原子锁定的维护配置。"""

    return HardwareProfile(
        profile_id="rail-cr7-maintenance",
        digest="profile-digest",
        mode=DeploymentMode.MAINTENANCE,
        backend=BackendKind.PLC,
        endpoint_ids=frozenset({"arm:cr7", "rail:a"}),
        interlock_mode=InterlockMode.OBSERVED_ONLY,
        commissioning_velocity_limit=0.25,
        commissioning_acceleration_limit=0.25,
    )


def _safety(*, rail: bool, arm: bool) -> SafetyInterlockObservation:
    """构造指定运动许可的新鲜互斥维护观测。"""

    return SafetyInterlockObservation(
        ObservationState.KNOWN,
        time.time(),
        1.0,
        "maintenance-interlock:test",
        True,
        False,
        rail,
        arm,
        True,
    )


class _Arm:
    """记录派发的机械臂模块替身，默认没有私有 Fence。"""

    endpoint_ids = frozenset({"arm:cr7"})

    def __init__(self) -> None:
        """初始化空派发记录和干净私有账本。"""

        self.executed_command_ids: list[str] = []
        self.has_unsettled_fence = False

    def execute(self, command: RobotCommand) -> CommandResult:
        """记录私有命令身份并返回成功。"""

        self.executed_command_ids.append(command.command_id)
        return CommandResult(command.command_id, CommandState.SUCCEEDED, "arm done")

    def request_controlled_stop(self, command_id: str, reason: str) -> CommandResult:
        """返回未使用的停止结果；本组用例不应触发遏制。"""

        del reason
        return CommandResult(command_id, CommandState.CANCELED, "arm stop")

    def settle_unknown(self, result: CommandResult, evidence: Any) -> CommandResult:
        """本组用例不结算私有 UNKNOWN，调用即视为测试失败。"""

        del result, evidence
        raise AssertionError("外部 Fence 测试不应结算机械臂私有账本")


class _Rail:
    """记录移动派发的导轨模块替身。"""

    endpoint_ids = frozenset({"rail:a"})
    allowed_targets = frozenset({"S04"})

    def __init__(self) -> None:
        """初始化空移动记录。"""

        self.moves: list[tuple[str, str]] = []

    def move_and_settle(
        self, command_id: str, target_ref: str
    ) -> RailStateObservation:
        """记录移动并返回精确完成见证。"""

        self.moves.append((command_id, target_ref))
        return RailStateObservation(
            ObservationState.KNOWN,
            time.time(),
            1.0,
            "rail:test",
            1.0,
            False,
            True,
            target_ref,
            command_id,
        )

    def observe(self) -> RailStateObservation:
        """返回已知零速观测。"""

        return RailStateObservation(
            ObservationState.KNOWN,
            time.time(),
            1.0,
            "rail:test",
            1.0,
            False,
            True,
            "S04",
            "observed-command",
        )

    def request_controlled_stop(self, command_id: str, reason: str) -> bool:
        """返回停止确认；本组用例不依赖该通道。"""

        del command_id, reason
        return True


def _coordinator(
    external_fence: Any = _UNSET,
) -> tuple[RailMountedArmCoordinator, _Arm, _Rail, InMemoryCommandJournal]:
    """装配组合协调器；未显式传入时不构造 external_fence 参数。"""

    arm = _Arm()
    rail = _Rail()
    journal = InMemoryCommandJournal()
    observations = iter(
        (
            _safety(rail=True, arm=False),
            _safety(rail=False, arm=True),
        )
    )
    kwargs = {} if external_fence is _UNSET else {"external_fence": external_fence}
    coordinator = RailMountedArmCoordinator(
        arm=arm,
        rail=rail,
        profile=_profile(),
        journal=journal,
        safety_observation=lambda: next(observations),
        **kwargs,
    )
    return coordinator, arm, rail, journal


def test_without_external_fence_dispatch_behaviour_is_unchanged() -> None:
    """不注入外部 Fence 时，协调器仍按原路径完成 rail-then-arm。"""

    coordinator, arm, rail, journal = _coordinator()

    result = coordinator.execute(_command("plain-1"), rail_target_ref="S04")

    assert result.state is CommandState.SUCCEEDED
    assert rail.moves == [("plain-1", "S04")]
    assert arm.executed_command_ids == ["plain-1:arm"]
    assert journal.get("plain-1") is not None
    assert coordinator.has_unsettled_fence is False


def test_external_fence_rejects_new_dispatch_before_rail_moves() -> None:
    """OS 权威 Fence 非空时，任何新派发都在移动导轨前被拒绝。"""

    coordinator, arm, rail, journal = _coordinator(
        external_fence=lambda: ("os:command-unknown",)
    )

    result = coordinator.execute(_command("blocked-1"), rail_target_ref="S04")

    assert result.state is CommandState.REJECTED
    assert "os:command-unknown" in result.message
    assert rail.moves == []
    assert arm.executed_command_ids == []
    assert journal.get("blocked-1") is None
    assert coordinator.has_unsettled_fence is True


def test_cleared_external_fence_allows_execution() -> None:
    """外部 Fence 清空后，同一命令身份可以重新派发并完成。"""

    fences: list[str] = ["os:command-unknown"]
    coordinator, arm, rail, journal = _coordinator(external_fence=lambda: fences)

    blocked = coordinator.execute(_command("after-clear"), rail_target_ref="S04")
    assert blocked.state is CommandState.REJECTED
    assert fences == ["os:command-unknown"]
    assert rail.moves == []

    fences.clear()
    assert coordinator.has_unsettled_fence is False
    result = coordinator.execute(_command("after-clear"), rail_target_ref="S04")

    assert result.state is CommandState.SUCCEEDED
    assert rail.moves == [("after-clear", "S04")]
    assert arm.executed_command_ids == ["after-clear:arm"]
    assert journal.get("after-clear") is not None


def test_external_fence_read_failure_rejects_dispatch() -> None:
    """读取外部 Fence 抛错时按存在未结算 Fence 拒绝，而不是继续派发。"""

    def broken() -> tuple[str, ...]:
        """模拟 OS Fence 存储不可用。"""

        raise ConnectionError("os fence store unavailable")

    coordinator, arm, rail, journal = _coordinator(external_fence=broken)

    result = coordinator.execute(_command("fence-read-failed"), rail_target_ref="S04")

    assert result.state is CommandState.REJECTED
    assert "OS 权威 Fence" in result.message
    assert "os fence store unavailable" in result.message
    assert rail.moves == []
    assert arm.executed_command_ids == []
    assert journal.get("fence-read-failed") is None
    assert coordinator.has_unsettled_fence is True
