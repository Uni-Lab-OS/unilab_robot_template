"""Headless MoveIt 调试 Adapter 的公共行为测试。"""

from __future__ import annotations

import time
from collections.abc import Mapping, Sequence
from typing import Any

from unilab_arm_cr7 import MODEL_DESCRIPTOR
from unilab_arm_cr7.adapters import MoveItCommissioningAdapter
from unilab_robot_contracts import (
    AngleUnit,
    CommandState,
    CommissioningPoseInput,
    ControlledStopCommand,
    EulerRotationOrder,
    JointJogCommand,
    MotionDirection,
    MovePoseCommand,
    MoveTargetCommand,
    ResolvedJointTarget,
    TcpAxis,
    TcpJogCommand,
)


class FakeCommissioningMoveGroup:
    """不启动 ROS/RViz 的确定性 MoveGroup 测试端口。"""

    def __init__(self) -> None:
        """创建零位六轴状态与空回执表。"""

        self.joints = [0.0] * 6
        self.pose = {
            "frame_ref": "arm_base",
            "xyz_m": [0.4, 0.0, 0.3],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        }
        self.results: dict[str, dict[str, Any]] = {}
        self.idle = True
        self.active_command_id: str | None = None
        self.fail_next_joint = False

    def read_commissioning_state(self) -> Mapping[str, Any]:
        """返回新鲜、空闲且包含完整关节/TCP 的状态。"""

        return {
            "observed_at": time.time(),
            "max_age_s": 1.0,
            "source": "test:headless-move-group",
            "online": True,
            "idle": self.idle,
            "active_command_id": self.active_command_id,
            "execution_fenced": False,
            "joint_positions": list(self.joints),
            "tcp_pose": dict(self.pose),
        }

    def execute_joint_target(
        self,
        *,
        group_name: str,
        joint_names: Sequence[str],
        target: Sequence[float],
        command_id: str,
        parameters: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """记录六轴目标并返回精确完成见证。"""

        del group_name, joint_names, parameters
        self.joints = [float(value) for value in target]
        if self.fail_next_joint:
            self.fail_next_joint = False
            raise RuntimeError("transport interrupted after dispatch")
        receipt = {"command_id": command_id, "state": "succeeded", "completed": True}
        self.results[command_id] = receipt
        return receipt

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
        """记录绝对 TCP 目标并返回精确完成见证。"""

        del group_name, parameters
        self.pose = {
            "frame_ref": frame_ref,
            "xyz_m": [float(value) for value in xyz_m],
            "orientation_xyzw": [float(value) for value in orientation_xyzw],
        }
        receipt = {"command_id": command_id, "state": "succeeded", "completed": True}
        self.results[command_id] = receipt
        return receipt

    def query_command(self, command_id: str) -> Mapping[str, Any] | None:
        """查询当前进程中的完成见证。"""

        return self.results.get(command_id)

    def cancel(self, command_id: str) -> bool:
        """测试端口确认停止。"""

        del command_id
        return True

    def apply_tool_context(self, tool_context: Any) -> Mapping[str, Any]:
        """确认仿真 PlanningScene 使用同一工具摘要和代次。"""

        return {
            "applied": True,
            "tool_context_digest": tool_context.digest,
            "attachment_generation": tool_context.attachment_generation,
        }


def test_move_target_uses_versioned_point_and_low_speed_profile() -> None:
    """维护目标移动必须校验版本/profile，并走无 RViz MoveGroup 端口。"""

    port = FakeCommissioningMoveGroup()
    adapter = MoveItCommissioningAdapter(
        port=port,
        model=MODEL_DESCRIPTOR,
        targets={
            "position1.approach": ResolvedJointTarget(
                "position1.approach",
                (0.1, -0.2, 0.3, 0.0, 0.2, 0.0),
                ("position1.approach",),
            )
        },
        point_set_revision="cr7-moveit-sim@1.0.0",
        hardware_profile_digest="profile-digest",
        tool_context_digest="tool-digest",
    )
    command = MoveTargetCommand(
        command_id="commissioning-target-1",
        hardware_profile_digest="profile-digest",
        source_boot_id="boot-1",
        monotonic_sequence=1,
        motion_profile_ref="maintenance-slow",
        velocity_scale=0.1,
        acceleration_scale=0.1,
        target_ref="position1.approach",
        target_revision="cr7-moveit-sim@1.0.0",
    )

    result = adapter.execute_commissioning(command)

    assert result.state is CommandState.SUCCEEDED
    assert port.joints == [0.1, -0.2, 0.3, 0.0, 0.2, 0.0]
    assert adapter.commissioning_snapshot().is_fresh()


def test_joint_jog_changes_only_selected_exact_model_joint() -> None:
    """单关节点动从完整状态构造新目标，并保持其余五轴不变。"""

    port = FakeCommissioningMoveGroup()
    adapter = MoveItCommissioningAdapter(
        port=port,
        model=MODEL_DESCRIPTOR,
        targets={},
        point_set_revision="cr7-moveit-sim@1.0.0",
        hardware_profile_digest="profile-digest",
        tool_context_digest="tool-digest",
    )
    command = JointJogCommand(
        command_id="commissioning-joint-1",
        hardware_profile_digest="profile-digest",
        source_boot_id="boot-1",
        monotonic_sequence=2,
        motion_profile_ref="maintenance-slow",
        velocity_scale=0.05,
        acceleration_scale=0.05,
        joint_ref="cr7_joint_2",
        direction=MotionDirection.NEGATIVE,
        step_si=0.02,
    )

    result = adapter.execute_commissioning(command)

    assert result.state is CommandState.SUCCEEDED
    assert port.joints == [0.0, -0.02, 0.0, 0.0, 0.0, 0.0]


def test_move_pose_uses_normalized_absolute_pose_and_tool_digest() -> None:
    """临时 mm/deg 位姿只以规范 m/quaternion 进入 MoveGroup。"""

    port = FakeCommissioningMoveGroup()
    adapter = MoveItCommissioningAdapter(
        port=port,
        model=MODEL_DESCRIPTOR,
        targets={},
        point_set_revision="cr7-moveit-sim@1.0.0",
        hardware_profile_digest="profile-digest",
        tool_context_digest="tool-digest",
    )
    command = MovePoseCommand(
        command_id="commissioning-pose-1",
        hardware_profile_digest="profile-digest",
        source_boot_id="boot-1",
        monotonic_sequence=3,
        motion_profile_ref="maintenance-slow",
        velocity_scale=0.05,
        acceleration_scale=0.05,
        pose_input=CommissioningPoseInput(
            frame_ref="arm_base",
            xyz_mm=(450.0, 20.0, 300.0),
            rotation_xyz=(0.0, 0.0, 90.0),
            angle_unit=AngleUnit.DEG,
            rotation_order=EulerRotationOrder.XYZ,
        ),
        tool_context_digest="tool-digest",
    )

    result = adapter.execute_commissioning(command)

    assert result.state is CommandState.SUCCEEDED
    assert port.pose["xyz_m"] == [0.45, 0.02, 0.3]


def test_tcp_jog_builds_one_finite_target_from_current_tcp() -> None:
    """TCP 点动读取当前位姿，只派发一次有限笛卡尔目标。"""

    port = FakeCommissioningMoveGroup()
    adapter = MoveItCommissioningAdapter(
        port=port,
        model=MODEL_DESCRIPTOR,
        targets={},
        point_set_revision="cr7-moveit-sim@1.0.0",
        hardware_profile_digest="profile-digest",
        tool_context_digest="tool-digest",
    )
    command = TcpJogCommand(
        command_id="commissioning-tcp-1",
        hardware_profile_digest="profile-digest",
        source_boot_id="boot-1",
        monotonic_sequence=4,
        motion_profile_ref="maintenance-slow",
        velocity_scale=0.05,
        acceleration_scale=0.05,
        frame_ref="tool",
        axis=TcpAxis.Z,
        direction=MotionDirection.POSITIVE,
        step_si=0.01,
    )

    result = adapter.execute_commissioning(command)

    assert result.state is CommandState.SUCCEEDED
    assert port.pose["xyz_m"] == [0.4, 0.0, 0.31]


def test_controlled_stop_is_allowed_while_motion_is_active() -> None:
    """受控停止必须针对活动命令工作，而不能被普通 idle 准入挡住。"""

    port = FakeCommissioningMoveGroup()
    port.idle = False
    port.active_command_id = "active-motion-1"
    adapter = MoveItCommissioningAdapter(
        port=port,
        model=MODEL_DESCRIPTOR,
        targets={},
        point_set_revision="cr7-moveit-sim@1.0.0",
        hardware_profile_digest="profile-digest",
        tool_context_digest="tool-digest",
    )
    command = ControlledStopCommand(
        command_id="stop-1",
        hardware_profile_digest="profile-digest",
        source_boot_id="boot-1",
        monotonic_sequence=5,
        target_command_id="active-motion-1",
        reason="operator requested stop",
    )

    result = adapter.execute_commissioning(command)

    assert result.state is CommandState.CANCELED


def test_ambiguous_dispatch_fences_following_commissioning_commands() -> None:
    """派发后通信中断必须进入 UNKNOWN，并阻断后续维护运动。"""

    port = FakeCommissioningMoveGroup()
    port.fail_next_joint = True
    target = ResolvedJointTarget(
        "position1.approach",
        (0.1, -0.2, 0.3, 0.0, 0.2, 0.0),
        ("position1.approach",),
    )
    adapter = MoveItCommissioningAdapter(
        port=port,
        model=MODEL_DESCRIPTOR,
        targets={target.target_ref: target},
        point_set_revision="cr7-moveit-sim@1.0.0",
        hardware_profile_digest="profile-digest",
        tool_context_digest="tool-digest",
    )
    first = MoveTargetCommand(
        "unknown-1",
        "profile-digest",
        "boot-1",
        10,
        "maintenance-slow",
        0.05,
        0.05,
        target.target_ref,
        "cr7-moveit-sim@1.0.0",
    )
    second = MoveTargetCommand(
        "blocked-2",
        "profile-digest",
        "boot-1",
        11,
        "maintenance-slow",
        0.05,
        0.05,
        target.target_ref,
        "cr7-moveit-sim@1.0.0",
    )

    unknown = adapter.execute_commissioning(first)
    blocked = adapter.execute_commissioning(second)

    assert unknown.state is CommandState.EXECUTION_UNKNOWN
    assert adapter.commissioning_snapshot().execution_fenced is True
    assert blocked.state is CommandState.REJECTED


def test_invalid_point_revision_is_rejected_before_dispatch() -> None:
    """活动点位版本不匹配是确定性准入拒绝，不得误报执行不明。"""

    port = FakeCommissioningMoveGroup()
    target = ResolvedJointTarget(
        "position1.approach",
        (0.1, -0.2, 0.3, 0.0, 0.2, 0.0),
        ("position1.approach",),
    )
    adapter = MoveItCommissioningAdapter(
        port=port,
        model=MODEL_DESCRIPTOR,
        targets={target.target_ref: target},
        point_set_revision="points@1.0.0",
        hardware_profile_digest="profile-digest",
        tool_context_digest="tool-digest",
    )
    command = MoveTargetCommand(
        "bad-revision",
        "profile-digest",
        "boot-1",
        12,
        "maintenance-slow",
        0.05,
        0.05,
        target.target_ref,
        "points@2.0.0",
    )

    result = adapter.execute_commissioning(command)

    assert result.state is CommandState.REJECTED
    assert port.joints == [0.0] * 6


def test_confirmed_stop_clears_target_commissioning_fence() -> None:
    """控制器确认停止后，可解除该活动调试命令的本地 Fence。"""

    port = FakeCommissioningMoveGroup()
    target = ResolvedJointTarget(
        "position1.approach",
        (0.1, -0.2, 0.3, 0.0, 0.2, 0.0),
        ("position1.approach",),
    )
    adapter = MoveItCommissioningAdapter(
        port=port,
        model=MODEL_DESCRIPTOR,
        targets={target.target_ref: target},
        point_set_revision="points@1.0.0",
        hardware_profile_digest="profile-digest",
        tool_context_digest="tool-digest",
    )
    port.fail_next_joint = True
    unknown = adapter.execute_commissioning(
        MoveTargetCommand(
            "active-unknown",
            "profile-digest",
            "boot-1",
            12,
            "maintenance-slow",
            0.05,
            0.05,
            target.target_ref,
            "points@1.0.0",
        )
    )
    assert unknown.state is CommandState.EXECUTION_UNKNOWN
    port.idle = False
    port.active_command_id = "active-unknown"
    result = adapter.execute_commissioning(
        ControlledStopCommand(
            "stop-unknown",
            "profile-digest",
            "boot-1",
            13,
            "active-unknown",
            "operator recovery",
        )
    )

    assert result.state is CommandState.CANCELED
    assert adapter.commissioning_snapshot().execution_fenced is False
