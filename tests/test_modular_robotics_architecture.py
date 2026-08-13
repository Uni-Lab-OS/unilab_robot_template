"""机械臂、导轨、WorkCell 和三后端共享合同的高层行为测试。"""

from __future__ import annotations

import time
from collections.abc import Mapping, Sequence
from dataclasses import replace
from typing import Any

import pytest
from unilab_arm_cr7 import ArmModule, StandaloneArmDevice
from unilab_arm_cr7.adapters.moveit import MoveItBackend
from unilab_arm_cr7.adapters.plc import PLCBackend, PLCProgramBinding
from unilab_arm_cr7.adapters.tcp_sdk import TcpSdkBackend
from unilab_rail_linear import RailModule
from unilab_rail_linear.adapters.plc import PLCRailAxisPort, PLCRailBinding
from unilab_rail_linear.adapters.simulation import SimulationRailAxisPort
from unilab_rail_mounted_arm import (
    EndpointLeaseRegistry,
    RailMountedArmCoordinator,
)
from unilab_robot_contracts import (
    ActionKind,
    BackendKind,
    BackendStatus,
    CommandRejectedError,
    CommandResult,
    CommandState,
    DeploymentMode,
    DispatchUnknownError,
    EndEffectorObservation,
    HardwareProfile,
    InMemoryCommandJournal,
    InterlockMode,
    MotionSegment,
    ObservationState,
    PhysicalSettlementEvidence,
    RailStateObservation,
    RobotCommand,
    SafetyInterlockObservation,
    CartesianPose,
    ResolvedCartesianTarget,
    ResolvedJointTarget,
    SQLiteCommandJournal,
)


def _command(command_id: str = "cmd-1") -> RobotCommand:
    """构造不含 Site/Material/地址的固定测试命令。"""

    return RobotCommand(
        command_id=command_id,
        action=ActionKind.PICK,
        hardware_profile_digest="profile-digest",
        payload_profile="beaker_500ml@v1",
        source_boot_id="test-boot",
        monotonic_sequence=1,
        segments=(MotionSegment("approach", "S04.pick.approach"),),
    )


def _profile(*, endpoints: frozenset[str] | None = None) -> HardwareProfile:
    """构造满足生产硬件互锁门禁的固定配置。"""

    return HardwareProfile(
        profile_id="rail-cr7-production",
        digest="profile-digest",
        mode=DeploymentMode.PRODUCTION,
        backend=BackendKind.PLC,
        endpoint_ids=endpoints or frozenset({"arm:cr7", "rail:a"}),
        interlock_mode=InterlockMode.VALIDATED_HARDWARE_INTERLOCK,
    )


def _safety(
    *, rail: bool, arm: bool, hardware: bool = True
) -> SafetyInterlockObservation:
    """构造新鲜的硬件互锁相位观测。"""

    return SafetyInterlockObservation(
        ObservationState.KNOWN,
        time.time(),
        1.0,
        "safety-plc:test",
        True,
        hardware,
        rail,
        arm,
        True,
    )


class _RecordingBackend:
    """记录机械臂派发顺序的测试后端。"""

    endpoint_ids = frozenset({"arm:cr7"})

    def __init__(self, events: list[str]) -> None:
        """注入共享事件列表。"""

        self.events = events

    def status(self) -> BackendStatus:
        """始终报告在线空闲。"""

        return BackendStatus(True, True)

    def execute(self, command: RobotCommand) -> CommandResult:
        """记录 arm 事件并返回成功见证。"""

        self.events.append(f"arm:start:{command.command_id}")
        return CommandResult(
            command.command_id, CommandState.SUCCEEDED, "arm completed"
        )

    def reconcile(self, command_id: str) -> CommandResult:
        """返回无法证明结果的测试对账。"""

        return CommandResult(command_id, CommandState.EXECUTION_UNKNOWN, "unknown")

    def request_stop(self, command_id: str, reason: str) -> CommandResult:
        """返回已取消的测试结果。"""

        return CommandResult(command_id, CommandState.CANCELED, reason)

    def end_effector_observation(self) -> EndEffectorObservation:
        """返回已知未持物的末端观测。"""

        return EndEffectorObservation(
            ObservationState.KNOWN, time.time(), 1.0, "test", False
        )


def test_robot_command_rejects_site_or_raw_address_leak() -> None:
    """RobotCommand 必须拒绝 Site、Material 和原始地址泄漏。"""

    with pytest.raises(ValueError, match="泄漏"):
        replace(_command(), metadata={"presence_variable": "ns=2;s=PLC.Sensor"})


def test_production_profile_rejects_observed_only_interlock() -> None:
    """普通遥测不能把 production profile 标记为可投产。"""

    with pytest.raises(ValueError, match="validated_hardware_interlock"):
        HardwareProfile(
            profile_id="unsafe",
            digest="digest",
            mode=DeploymentMode.PRODUCTION,
            backend=BackendKind.PLC,
            endpoint_ids=frozenset({"arm:cr7"}),
            interlock_mode=InterlockMode.OBSERVED_ONLY,
        )


def test_standalone_production_rejects_non_hardware_safety_observation() -> None:
    """即使 profile 名义合规，单机械臂也不得接受普通遥测伪装的生产许可。"""

    events: list[str] = []
    arm = ArmModule(
        backend=_RecordingBackend(events),
        profile=_profile(endpoints=frozenset({"arm:cr7"})),
        journal=InMemoryCommandJournal(),
        safety_observation=lambda: _safety(rail=False, arm=True, hardware=False),
    )

    result = arm.execute(_command("standalone-unsafe"))

    assert result.state is CommandState.REJECTED
    assert "硬件互锁" in result.message
    assert events == []


def test_workcell_orders_rail_then_arm_and_releases_fence() -> None:
    """WorkCell 必须先见证导轨稳定，再派发机械臂，成功后释放 Fence。"""

    events: list[str] = []
    rail = RailModule(
        port=SimulationRailAxisPort(endpoint_id="rail:a", events=events),
        allowed_targets=frozenset({"S04"}),
    )

    def safety() -> SafetyInterlockObservation:
        """根据导轨 settled 事件模拟硬件许可相位切换。"""

        settled = any(event.startswith("rail:settled") for event in events)
        return _safety(rail=not settled, arm=settled)

    arm = ArmModule(
        backend=_RecordingBackend(events),
        profile=_profile(),
        journal=InMemoryCommandJournal(),
        safety_observation=safety,
    )
    journal = InMemoryCommandJournal()
    coordinator = RailMountedArmCoordinator(
        arm=arm,
        rail=rail,
        profile=_profile(),
        journal=journal,
        safety_observation=safety,
    )

    result = coordinator.execute(_command(), rail_target_ref="S04")

    assert result.state is CommandState.SUCCEEDED
    assert events == [
        "rail:start:cmd-1:S04",
        "rail:settled:cmd-1:S04",
        "arm:start:cmd-1:arm",
    ]
    assert journal.is_fenced("cmd-1") is False


class _UnknownRailPort:
    """模拟已派发但无法证明导轨停止的端口。"""

    endpoint_ids = frozenset({"rail:a"})

    def move(self, command_id: str, target_ref: str) -> None:
        """接受移动但不建立完成见证。"""

    def observe(self) -> RailStateObservation:
        """返回 unknown，代表现场物理状态不明。"""

        return RailStateObservation(
            ObservationState.UNKNOWN, time.time(), 1.0, "test", None, None, None
        )

    def request_stop(self, command_id: str) -> bool:
        """无法确认普通停止。"""

        return False


def test_unknown_rail_never_dispatches_arm_and_keeps_fence() -> None:
    """导轨状态不明时不得派发机械臂，公共命令保持 execution_unknown 与 Fence。"""

    events: list[str] = []
    arm = ArmModule(
        backend=_RecordingBackend(events),
        profile=_profile(),
        journal=InMemoryCommandJournal(),
        safety_observation=lambda: _safety(rail=False, arm=True),
    )
    journal = InMemoryCommandJournal()
    coordinator = RailMountedArmCoordinator(
        arm=arm,
        rail=RailModule(port=_UnknownRailPort(), allowed_targets=frozenset({"S04"})),
        profile=_profile(),
        journal=journal,
        safety_observation=lambda: _safety(rail=True, arm=False),
    )

    result = coordinator.execute(_command(), rail_target_ref="S04")

    assert result.state is CommandState.EXECUTION_UNKNOWN
    assert events == []
    assert journal.is_fenced("cmd-1") is True


@pytest.mark.parametrize("persistent", [False, True])
def test_unknown_barrier_requires_explicit_physical_settlement(
    tmp_path: Any, persistent: bool
) -> None:
    """普通 update 不得把 UNKNOWN 改成终态并偷偷解除阻断。"""

    journal = (
        SQLiteCommandJournal(tmp_path / "commands.sqlite3")
        if persistent
        else InMemoryCommandJournal()
    )
    command = _command("unknown-settlement")
    journal.accept(command)
    journal.update(
        CommandResult(
            command.command_id,
            CommandState.EXECUTION_UNKNOWN,
            "dispatch uncertain",
        )
    )
    terminal = CommandResult(
        command.command_id,
        CommandState.FAILED,
        "operator observed stopped",
    )

    with pytest.raises(CommandRejectedError, match="非法命令状态迁移"):
        journal.update(terminal, fenced=False)
    assert journal.is_fenced(command.command_id) is True

    journal.settle(
        terminal,
        PhysicalSettlementEvidence(
            command.command_id,
            CommandState.FAILED,
            "plc-cycle-41",
            "operator-confirmed-plc-history",
        ),
    )
    assert journal.is_fenced(command.command_id) is False


def test_duplicate_public_endpoint_activation_is_rejected() -> None:
    """同一真实机械臂不能同时注册 standalone 与 WorkCell 公共入口。"""

    registry = EndpointLeaseRegistry()
    lease = registry.acquire("standalone-cr7", frozenset({"arm:cr7"}))
    with pytest.raises(Exception, match="已被公共 Device 激活"):
        registry.acquire("rail-cr7-workcell", frozenset({"arm:cr7", "rail:a"}))
    lease.close()


class _MoveGroup:
    """记录 MoveIt 六轴执行参数的 headless 测试 port。"""

    def __init__(self) -> None:
        """创建空调用记录。"""

        self.calls: list[tuple[str, tuple[float, ...]]] = []

    def execute_joint_target(
        self,
        *,
        group_name: str,
        joint_names: Sequence[str],
        target: Sequence[float],
        command_id: str,
        parameters: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """记录六轴目标并模拟 move_group 成功。"""

        self.calls.append(("joint", tuple(target)))
        return {
            "command_id": command_id,
            "state": "succeeded",
            "completed": True,
        }

    def execute_cartesian_target(
        self,
        *,
        group_name: str,
        frame_ref: str,
        xyz_m: Sequence[float],
        orientation_xyzw: Sequence[float],
        command_id: str,
        parameters: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """记录绝对笛卡尔目标并模拟 move_group 成功。"""

        del group_name, frame_ref, orientation_xyzw, parameters
        self.calls.append(("cartesian", tuple(xyz_m)))
        return {
            "command_id": command_id,
            "state": "succeeded",
            "completed": True,
        }

    def query_command(self, command_id: str) -> Mapping[str, Any] | None:
        """返回成功对账。"""

        return {
            "command_id": command_id,
            "state": "succeeded",
            "completed": True,
            "message": "done",
        }

    def cancel(self, command_id: str) -> bool:
        """模拟确认取消。"""

        return True


def test_moveit_is_six_axis_and_headless() -> None:
    """MoveIt 后端只接受六轴点位，运行组件不含 RViz。"""

    with pytest.raises(ValueError, match="六轴"):
        MoveItBackend(
            port=_MoveGroup(),
            endpoint_ids=frozenset({"arm:cr7"}),
            targets={
                "bad": ResolvedJointTarget(
                    "bad",
                    tuple(float(value) for value in range(7)),
                    ("bad",),
                )
            },
        )
    port = _MoveGroup()
    backend = MoveItBackend(
        port=port,
        endpoint_ids=frozenset({"arm:cr7"}),
        targets={
            "S04.pick.approach": ResolvedJointTarget(
                "S04.pick.approach",
                (0.0,) * 6,
                ("S04.pick.approach",),
            ),
            "S04.pick.interaction": ResolvedCartesianTarget(
                "S04.pick.interaction",
                CartesianPose(
                    "arm_base",
                    (0.4, 0.1, 0.25),
                    (0.0, 0.0, 0.0, 1.0),
                ),
                ("S04.pick.approach", "S04.pick.interaction"),
                (0.0,) * 6,
            ),
        },
    )

    result = backend.execute(_command())

    assert result.state is CommandState.SUCCEEDED
    assert port.calls == [("joint", (0.0,) * 6)]
    assert "rviz" not in " ".join(backend.required_launch_components()).lower()


def test_moveit_executes_pre_resolved_cartesian_target() -> None:
    """MoveIt Adapter 只能接收解析器生成的绝对笛卡尔目标。"""

    port = _MoveGroup()
    backend = MoveItBackend(
        port=port,
        endpoint_ids=frozenset({"arm:cr7"}),
        targets={
            "S04.pick.approach": ResolvedCartesianTarget(
                "S04.pick.approach",
                CartesianPose(
                    "arm_base",
                    (0.4, 0.1, 0.25),
                    (0.0, 0.0, 0.0, 1.0),
                ),
                ("S04.pick.origin", "S04.pick.approach"),
                None,
            )
        },
    )

    result = backend.execute(_command())

    assert result.state is CommandState.SUCCEEDED
    assert port.calls == [("cartesian", (0.4, 0.1, 0.25))]


def test_standalone_arm_reuses_arm_module_without_rail() -> None:
    """单机械臂部署不加载 RailModule，仍复用同一 ArmModule 和公共动作名。"""

    events: list[str] = []
    profile = _profile(endpoints=frozenset({"arm:cr7"}))
    arm = ArmModule(
        backend=_RecordingBackend(events),
        profile=profile,
        journal=InMemoryCommandJournal(),
        safety_observation=lambda: _safety(rail=False, arm=True),
    )
    standalone = StandaloneArmDevice(
        arm, lambda action, **arguments: _command(str(arguments["command_id"]))
    )

    result = standalone.pick(command_id="standalone-1")

    assert result.state is CommandState.SUCCEEDED
    assert events == ["arm:start:standalone-1"]


class _PLCVariables:
    """记录 PLC 读写并提供可控完成见证的测试端口。"""

    def __init__(self) -> None:
        """初始化允许写入、完成码清零且 Robot_Home 有效的现场镜像。"""

        self.values: dict[str, Any] = {
            "allowed": True,
            "complete": 0,
            "home": True,
            "rail_ack": "old-command",
            "rail_completed": "old-command",
            "rail_settled": True,
            "rail_position": 1.0,
            "rail_moving": False,
        }
        self.writes: list[tuple[str, Any]] = []

    def read(self, variable: str) -> Any:
        """返回指定 PLC 节点的当前测试值。"""

        return self.values[variable]

    def write(self, variable: str, value: Any) -> None:
        """记录并应用一次 PLC 写入。"""

        self.values[variable] = value
        self.writes.append((variable, value))

    def wait_equal(self, variable: str, expected: Any, timeout_s: float) -> bool:
        """模拟等待；机器人完成/Home 可到达，导轨命令身份必须精确匹配。"""

        del timeout_s
        if variable in {"complete", "home"}:
            return True
        return self.values.get(variable) == expected


def test_plc_backend_requires_a_fresh_completion_cycle() -> None:
    """PLC 后端必须先看到完成码为零，禁止把上一轮相同任务号当作新见证。"""

    port = _PLCVariables()
    backend = PLCBackend(
        port=port,
        endpoint_ids=frozenset({"arm:cr7"}),
        programs={
            "S04.pick.approach": PLCProgramBinding(
                7,
                "task",
                "write_done",
                "complete",
                "home",
                "allowed",
            )
        },
    )

    result = backend.execute(_command("plc-fresh"))
    assert result.state is CommandState.SUCCEEDED
    assert ("task", 7) in port.writes

    port.values["complete"] = 7
    with pytest.raises(CommandRejectedError, match="上一轮完成码"):
        backend.execute(_command("plc-stale"))


def test_plc_backend_prevalidates_all_segments_before_first_write() -> None:
    """后续段配置错误必须在首次物理写入前拒绝。"""

    port = _PLCVariables()
    backend = PLCBackend(
        port=port,
        endpoint_ids=frozenset({"arm:cr7"}),
        programs={
            "valid": PLCProgramBinding(
                7, "task", "write_done", "complete", "home", "allowed"
            )
        },
    )
    command = replace(
        _command("plc-prevalidate"),
        segments=(
            MotionSegment("first", "valid"),
            MotionSegment("second", "missing"),
        ),
    )

    with pytest.raises(CommandRejectedError, match="missing"):
        backend.execute(command)
    assert port.writes == []


def test_plc_rail_rejects_stale_settled_without_command_ack() -> None:
    """导轨即使仍显示 settled，也必须确认本轮 command_id 后才能被接受。"""

    port = _PLCVariables()
    axis = PLCRailAxisPort(
        port=port,
        binding=PLCRailBinding(
            target_variable="rail_target",
            command_id_variable="rail_command",
            accepted_command_id_variable="rail_ack",
            completed_command_id_variable="rail_completed",
            start_variable="rail_start",
            moving_variable="rail_moving",
            settled_variable="rail_settled",
            position_variable="rail_position",
            target_values={"S04": 1.0},
        ),
        endpoint_ids=frozenset({"rail:a"}),
    )

    with pytest.raises(TimeoutError, match="命令身份确认"):
        axis.move("new-command", "S04")


def test_plc_rail_requires_exact_completed_command_identity() -> None:
    """导轨已接受但没有本轮完成 command_id 时不得结算。"""

    port = _PLCVariables()
    port.values["rail_ack"] = "new-command"
    axis = PLCRailAxisPort(
        port=port,
        binding=PLCRailBinding(
            target_variable="rail_target",
            command_id_variable="rail_command",
            accepted_command_id_variable="rail_ack",
            completed_command_id_variable="rail_completed",
            start_variable="rail_start",
            moving_variable="rail_moving",
            settled_variable="rail_settled",
            position_variable="rail_position",
            target_values={"S04": 1.0},
        ),
        endpoint_ids=frozenset({"rail:a"}),
    )

    with pytest.raises(TimeoutError, match="本轮完成回执"):
        axis.move("new-command", "S04")


class _SDKPort:
    """返回确定完成见证的厂家 TCP/SDK 测试端口。"""

    def __init__(self) -> None:
        """创建空调用记录。"""

        self.calls: list[tuple[str, str]] = []

    def execute_target(
        self, target_ref: str, parameters: Mapping[str, Any], command_id: str
    ) -> Mapping[str, Any]:
        """记录白名单目标并返回厂家完成回执。"""

        del parameters
        self.calls.append((command_id, target_ref))
        return {
            "command_id": command_id,
            "state": "succeeded",
            "completed": True,
        }

    def query_command(self, command_id: str) -> Mapping[str, Any] | None:
        """返回同一命令的成功对账结果。"""

        return {
            "command_id": command_id,
            "state": "succeeded",
            "completed": True,
        }

    def request_stop(self, command_id: str, reason: str) -> bool:
        """模拟厂家已确认普通停止。"""

        del command_id, reason
        return True


def test_tcp_sdk_backend_uses_the_same_robot_command() -> None:
    """TCP/SDK 后端直接复用 RobotCommand，不引入后端特化公开动作。"""

    port = _SDKPort()
    backend = TcpSdkBackend(port=port, endpoint_ids=frozenset({"arm:cr7"}))

    result = backend.execute(_command("sdk-1"))

    assert result.state is CommandState.SUCCEEDED
    assert port.calls == [("sdk-1", "S04.pick.approach")]


def test_tcp_sdk_backend_rejects_unbound_success_receipt_as_unknown() -> None:
    """SDK 只返回 succeeded 文本不能证明本次物理命令完成。"""

    class UnboundReceiptPort(_SDKPort):
        """故意缺少 command identity 的不安全 SDK 替身。"""

        def execute_target(
            self,
            target_ref: str,
            parameters: Mapping[str, Any],
            command_id: str,
        ) -> Mapping[str, Any]:
            """返回无法绑定的伪完成回执。"""

            del target_ref, parameters, command_id
            return {"state": "succeeded", "completed": True}

    backend = TcpSdkBackend(
        port=UnboundReceiptPort(), endpoint_ids=frozenset({"arm:cr7"})
    )

    with pytest.raises(DispatchUnknownError, match="command_id 不匹配"):
        backend.execute(_command("sdk-unbound"))
