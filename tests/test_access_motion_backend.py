"""D9-1S 机械臂、夹爪、快换和负载观测的单命令执行测试。"""

from __future__ import annotations

import time

import pytest
from unilab_robot_contracts import (
    ActionKind,
    BackendStatus,
    CommandRejectedError,
    CommandResult,
    CommandState,
    EndEffectorObservation,
    GripperObservation,
    MotionSegment,
    ObservationState,
    RigidTransform,
    RobotCommand,
    ToolAttachmentObservation,
    ToolContext,
)
from unilab_robot_runtime import AccessMotionBackend


class _ArmBackend:
    """记录每个机械臂阶段并返回确定完成。"""

    endpoint_ids = frozenset({"arm:test"})

    def __init__(self) -> None:
        """创建空阶段记录。"""

        self.targets: list[str] = []
        self.command_ids: list[str] = []

    def status(self) -> BackendStatus:
        """返回在线空闲。"""

        return BackendStatus(True, True)

    def execute(self, command: RobotCommand) -> CommandResult:
        """记录唯一机械臂段。"""

        self.targets.append(command.segments[0].target_ref)
        self.command_ids.append(command.command_id)
        return CommandResult(command.command_id, CommandState.SUCCEEDED, "arm done")

    def reconcile(self, command_id: str) -> CommandResult:
        """返回测试完成见证。"""

        return CommandResult(command_id, CommandState.SUCCEEDED, "arm done")

    def request_stop(self, command_id: str, reason: str) -> CommandResult:
        """返回不确定停止；理由只保持接口一致。"""

        del reason
        return CommandResult(command_id, CommandState.EXECUTION_UNKNOWN, "stopping")

    def end_effector_observation(self) -> EndEffectorObservation:
        """返回兼容的已知末端观测。"""

        return EndEffectorObservation(
            ObservationState.KNOWN,
            time.time(),
            1.0,
            "arm-test",
            None,
        )


class _ToolChanger:
    """持有一份与规划完全一致的快换上下文。"""

    def __init__(self, context: ToolContext, *, locked: bool = True) -> None:
        """冻结上下文和锁紧测试状态。"""

        self.context = context
        self.locked = locked

    def observe(self) -> ToolAttachmentObservation:
        """返回最新工具身份与附着代次。"""

        return ToolAttachmentObservation(
            ObservationState.KNOWN,
            time.time(),
            1.0,
            "tool-test",
            self.context.context_id,
            self.context.attachment_generation,
            self.locked,
        )

    def change_tool(self, command_id: str, *, tool_ref: str) -> CommandResult:
        """该测试不执行换装。"""

        del tool_ref
        return CommandResult(command_id, CommandState.SUCCEEDED, "unchanged")

    @property
    def active_tool_context(self) -> ToolContext:
        """返回规划器当前使用的同一上下文。"""

        return self.context


class _Gripper:
    """模拟抓取成功或失败，并提供负载观测。"""

    def __init__(self, *, fail_grip: bool = False) -> None:
        """从无负载状态启动。"""

        self.holding = False
        self.fail_grip = fail_grip

    def observe(self) -> GripperObservation:
        """返回最新负载状态。"""

        return GripperObservation(
            ObservationState.KNOWN,
            time.time(),
            1.0,
            "gripper-test",
            self.holding,
            self.holding,
        )

    def grip(self, command_id: str, *, payload_profile: str) -> CommandResult:
        """按配置成功持有负载或返回确定失败。"""

        del payload_profile
        if self.fail_grip:
            return CommandResult(command_id, CommandState.FAILED, "grip failed")
        self.holding = True
        return CommandResult(command_id, CommandState.SUCCEEDED, "gripped")

    def release(self, command_id: str) -> CommandResult:
        """释放测试负载。"""

        self.holding = False
        return CommandResult(command_id, CommandState.SUCCEEDED, "released")


def _command() -> RobotCommand:
    """返回最小但完整的 pick 接近运动块。"""

    return RobotCommand(
        "access-1",
        ActionKind.PICK,
        "a" * 64,
        "beaker@v1",
        "boot-1",
        1,
        (
            MotionSegment(
                "approach",
                "rack.r1c1.approach",
                {"phase_kind": "arm_move", "payload_state": "empty"},
            ),
            MotionSegment(
                "pick",
                "end_effector.grip",
                {"phase_kind": "end_effector", "payload_state": "loaded"},
            ),
            MotionSegment(
                "observe",
                "end_effector.payload.loaded",
                {"phase_kind": "observe", "payload_state": "loaded"},
            ),
            MotionSegment(
                "retract",
                "rack.r1c1.approach",
                {"phase_kind": "arm_move", "payload_state": "loaded"},
            ),
        ),
    )


def _backend(*, fail_grip: bool = False, locked: bool = True) -> AccessMotionBackend:
    """装配固定工具上下文的组合后端。"""

    context = ToolContext("gripper@1", "b" * 64, RigidTransform.identity(), 1)
    return AccessMotionBackend(
        arm_backend=_ArmBackend(),
        end_effector=_Gripper(fail_grip=fail_grip),
        tool_changer=_ToolChanger(context, locked=locked),
        expected_tool_context=context,
    )


def test_access_motion_executes_arm_gripper_observation_and_retract() -> None:
    """成功必须包含抓取后的负载观测和撤离。"""

    backend = _backend()
    result = backend.execute(_command())

    assert result.state is CommandState.SUCCEEDED
    assert backend.arm_backend.targets == [  # type: ignore[attr-defined]
        "rack.r1c1.approach",
        "rack.r1c1.approach",
    ]
    assert backend.arm_backend.command_ids == [  # type: ignore[attr-defined]
        "access-1:arm:approach",
        "access-1:arm:retract",
    ]


def test_access_motion_rejects_unlocked_tool_before_first_arm_move() -> None:
    """快换未锁紧时不得产生任何机械臂物理作用。"""

    backend = _backend(locked=False)
    with pytest.raises(CommandRejectedError, match="快换工具"):
        backend.execute(_command())
    assert backend.arm_backend.targets == []  # type: ignore[attr-defined]


def test_access_motion_keeps_unknown_after_gripper_failure_post_move() -> None:
    """机械臂已动后夹爪失败不能降级成可自动重试的普通失败。"""

    result = _backend(fail_grip=True).execute(_command())
    assert result.state is CommandState.EXECUTION_UNKNOWN
