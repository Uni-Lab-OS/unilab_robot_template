"""导轨机械臂组合包的软件侧应急遏制行为测试。"""

from __future__ import annotations

import time
from threading import Event, Thread
from typing import Any

import pytest
from unilab_rail_mounted_arm import RailMountedArmCoordinator
from unilab_robot_contracts import (
    ActionKind,
    BackendKind,
    CommandRejectedError,
    CommandResult,
    CommandState,
    DeploymentMode,
    DispatchUnknownError,
    HardwareProfile,
    InMemoryCommandJournal,
    InterlockMode,
    MotionSegment,
    ObservationState,
    PhysicalSettlementEvidence,
    RailStateObservation,
    RobotCommand,
    SafetyInterlockObservation,
)


def _command(command_id: str) -> RobotCommand:
    """构造只携带稳定资产引用的测试命令。"""

    return RobotCommand(
        command_id=command_id,
        action=ActionKind.PICK,
        hardware_profile_digest="profile-digest",
        payload_profile="beaker_500ml@v1",
        source_boot_id="containment-test",
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
    """构造新鲜且互斥的维护观测。"""

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
    """记录组合协调器公开行为的机械臂模块替身。"""

    endpoint_ids = frozenset({"arm:cr7"})
    def __init__(
        self,
        events: list[str],
        *,
        started: Event | None = None,
        release: Event | None = None,
    ) -> None:
        """注入事件列表和可选的并发执行闸门。"""

        self.events = events
        self.started = started
        self.release = release
        self.executed_command_ids: set[str] = set()
        self.has_unsettled_fence = False

    def execute(self, command: RobotCommand) -> CommandResult:
        """记录派发，并按需等待外部停止请求。"""

        self.events.append(f"arm:execute:{command.command_id}")
        self.executed_command_ids.add(command.command_id)
        if self.started is not None:
            self.started.set()
        if self.release is not None:
            assert self.release.wait(timeout=2.0)
        return CommandResult(command.command_id, CommandState.SUCCEEDED, "arm done")

    def request_controlled_stop(
        self, command_id: str, reason: str
    ) -> CommandResult:
        """记录受控停止请求；确认只作诊断，不作物理结算。"""

        self.events.append(f"arm:stop:{command_id}:{reason}")
        if command_id in self.executed_command_ids:
            self.has_unsettled_fence = True
        return CommandResult(command_id, CommandState.CANCELED, "arm stop confirmed")

    def settle_unknown(
        self,
        result: CommandResult,
        evidence: PhysicalSettlementEvidence,
    ) -> CommandResult:
        """校验私有身份并模拟可信物理结算。"""

        if result.command_id != evidence.command_id:
            raise CommandRejectedError("arm evidence mismatch")
        self.events.append(f"arm:settle:{result.command_id}:{evidence.witness_id}")
        self.has_unsettled_fence = False
        return result


class _Rail:
    """可配置派发歧义的导轨模块替身。"""

    endpoint_ids = frozenset({"rail:a"})
    allowed_targets = frozenset({"S04"})

    def __init__(self, events: list[str], *, unknown: bool = False) -> None:
        """注入事件列表，并可令移动结果进入 UNKNOWN。"""

        self.events = events
        self.unknown = unknown

    def move_and_settle(
        self, command_id: str, target_ref: str
    ) -> RailStateObservation:
        """返回精确完成见证，或模拟派发后失联。"""

        self.events.append(f"rail:move:{command_id}:{target_ref}")
        if self.unknown:
            raise DispatchUnknownError("rail communication lost")
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
        """记录导轨普通受控停止请求。"""

        self.events.append(f"rail:stop:{command_id}:{reason}")
        return True


def _coordinator(
    *,
    arm: _Arm,
    rail: _Rail,
    safety_observation: Any,
) -> tuple[RailMountedArmCoordinator, InMemoryCommandJournal]:
    """装配独立的组合协调器和命令账本。"""

    journal = InMemoryCommandJournal()
    return (
        RailMountedArmCoordinator(
            arm=arm,
            rail=rail,
            profile=_profile(),
            journal=journal,
            safety_observation=safety_observation,
        ),
        journal,
    )


def test_rail_dispatch_unknown_requests_both_stops_and_keeps_barrier() -> None:
    """导轨派发歧义后应遏制两个模块，且停止确认不能解除阻断。"""

    events: list[str] = []
    coordinator, journal = _coordinator(
        arm=_Arm(events),
        rail=_Rail(events, unknown=True),
        safety_observation=lambda: _safety(rail=True, arm=False),
    )

    result = coordinator.execute(_command("rail-unknown"), rail_target_ref="S04")

    assert result.state is CommandState.EXECUTION_UNKNOWN
    assert journal.is_fenced("rail-unknown") is True
    assert events == [
        "rail:move:rail-unknown:S04",
        "rail:stop:rail-unknown:导轨阶段结果不明",
        "arm:stop:rail-unknown:arm:导轨阶段结果不明",
    ]
    assert result.output["controlled_stop"]["physical_settlement"] is False
    assert result.output["controlled_stop"]["rail"]["confirmed"] is True
    assert result.output["controlled_stop"]["arm"]["confirmed"] is True

    blocked = coordinator.execute(_command("next-command"), rail_target_ref="S04")
    assert blocked.state is CommandState.REJECTED
    assert "未物理结算" in blocked.message


def test_interlock_loss_after_rail_settled_never_dispatches_arm() -> None:
    """导轨完成后的互锁切换失败必须停止两端，不能继续启动机械臂。"""

    events: list[str] = []
    observations = iter(
        (
            _safety(rail=True, arm=False),
            _safety(rail=True, arm=True),
        )
    )
    coordinator, journal = _coordinator(
        arm=_Arm(events),
        rail=_Rail(events),
        safety_observation=lambda: next(observations),
    )

    result = coordinator.execute(_command("interlock-loss"), rail_target_ref="S04")

    assert result.state is CommandState.EXECUTION_UNKNOWN
    assert journal.is_fenced("interlock-loss") is True
    assert not any(event.startswith("arm:execute") for event in events)
    assert any(event.startswith("rail:stop:interlock-loss") for event in events)
    assert any(event.startswith("arm:stop:interlock-loss:arm") for event in events)


def test_external_stop_during_arm_execution_cannot_be_overwritten_by_success() -> None:
    """外部停止与执行并发时，迟到的成功返回不得覆盖 UNKNOWN 阻断。"""

    events: list[str] = []
    started = Event()
    release = Event()
    arm = _Arm(events, started=started, release=release)
    observations = iter(
        (
            _safety(rail=True, arm=False),
            _safety(rail=False, arm=True),
        )
    )
    coordinator, journal = _coordinator(
        arm=arm,
        rail=_Rail(events),
        safety_observation=lambda: next(observations),
    )
    execution_result: list[CommandResult] = []
    worker = Thread(
        target=lambda: execution_result.append(
            coordinator.execute(_command("operator-stop"), rail_target_ref="S04")
        )
    )
    worker.start()
    assert started.wait(timeout=2.0)

    stop_result = coordinator.request_controlled_stop(
        "operator-stop", reason="operator requested containment"
    )
    release.set()
    worker.join(timeout=2.0)

    assert worker.is_alive() is False
    assert stop_result.state is CommandState.EXECUTION_UNKNOWN
    assert execution_result[0].state is CommandState.EXECUTION_UNKNOWN
    assert journal.is_fenced("operator-stop") is True
    assert any(event.startswith("arm:stop:operator-stop:arm") for event in events)
    assert any(event.startswith("rail:stop:operator-stop") for event in events)

    public_terminal = CommandResult(
        "operator-stop",
        CommandState.CANCELED,
        "operator verified workcell stopped",
    )
    public_evidence = PhysicalSettlementEvidence(
        "operator-stop",
        CommandState.CANCELED,
        "workcell-witness-1",
        "maintenance-inspection",
    )
    with pytest.raises(CommandRejectedError, match="私有物理见证"):
        coordinator.settle_unknown(public_terminal, public_evidence)
    assert journal.is_fenced("operator-stop") is True

    coordinator.settle_unknown(
        public_terminal,
        public_evidence,
        arm_result=CommandResult(
            "operator-stop:arm",
            CommandState.CANCELED,
            "operator verified arm stopped",
        ),
        arm_evidence=PhysicalSettlementEvidence(
            "operator-stop:arm",
            CommandState.CANCELED,
            "arm-witness-1",
            "maintenance-inspection",
        ),
    )
    assert journal.is_fenced("operator-stop") is False
    assert coordinator.has_unsettled_fence is False


def test_stop_request_failure_does_not_hide_original_unknown() -> None:
    """停止端口自身异常不得吞掉原始歧义，也不得解除本地派发阻断。"""

    class FailingStopRail(_Rail):
        """模拟受控停止写入也失败的导轨端口。"""

        def request_controlled_stop(self, command_id: str, reason: str) -> bool:
            """抛出停止通道异常。"""

            del command_id, reason
            raise ConnectionError("stop channel unavailable")

    events: list[str] = []
    coordinator, journal = _coordinator(
        arm=_Arm(events),
        rail=FailingStopRail(events, unknown=True),
        safety_observation=lambda: _safety(rail=True, arm=False),
    )

    result = coordinator.execute(_command("stop-failed"), rail_target_ref="S04")

    assert result.state is CommandState.EXECUTION_UNKNOWN
    assert journal.is_fenced("stop-failed") is True
    assert result.output["controlled_stop"]["rail"]["confirmed"] is False
    assert "stop channel unavailable" in result.output["controlled_stop"]["rail"]["error"]
    assert result.output["controlled_stop"]["physical_settlement"] is False
