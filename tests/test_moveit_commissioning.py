"""MoveIt 调试 Adapter 必须区分已知失败与派发不明 Fence。"""

from __future__ import annotations

import time
from collections.abc import Mapping, Sequence
from typing import Any

from unilab_arm_cr5 import MODEL_DESCRIPTOR
from unilab_arm_cr5.adapters import MoveItCommissioningAdapter
from unilab_robot_contracts import (
    CommandState,
    MoveTargetCommand,
    ResolvedJointTarget,
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


class _ScriptedJointPort:
    """按预定回执或异常响应 move_target，并保持新鲜空闲快照。"""

    def __init__(self, outcomes: list[object]) -> None:
        self._outcomes = list(outcomes)
        self.calls = 0

    def read_commissioning_state(self) -> dict[str, Any]:
        return {
            "observed_at": time.time(),
            "max_age_s": 1.0,
            "source": "test:moveit",
            "online": True,
            "idle": True,
            "active_command_id": None,
            "execution_fenced": False,
            "joint_positions": (0.0,) * len(MODEL_DESCRIPTOR.joint_specs),
            "tcp_pose": {
                "frame_ref": "arm_base",
                "xyz_m": [0.4, 0.0, 0.3],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
        }

    def execute_joint_target(self, **kwargs: Any) -> dict[str, Any]:
        self.calls += 1
        outcome = self._outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        receipt = dict(outcome)
        receipt.setdefault("command_id", kwargs["command_id"])
        return receipt


def _adapter(port: _ScriptedJointPort) -> MoveItCommissioningAdapter:
    target = ResolvedJointTarget(
        "collection.target-1.interaction_seed",
        (0.0,) * 6,
        ("collection.target-1.interaction_seed",),
    )
    return MoveItCommissioningAdapter(
        port=port,
        model=MODEL_DESCRIPTOR,
        targets={target.target_ref: target},
        point_set_revision="ptlc-cr5-rail@3.1.1",
        hardware_profile_digest="a" * 64,
        tool_context_digest="b" * 64,
    )


def _move_target(command_id: str) -> MoveTargetCommand:
    return MoveTargetCommand(
        command_id=command_id,
        hardware_profile_digest="a" * 64,
        source_boot_id="boot-1",
        monotonic_sequence=1,
        motion_profile_ref="commissioning",
        velocity_scale=0.2,
        acceleration_scale=0.2,
        target_ref="collection.target-1.interaction_seed",
        target_revision="ptlc-cr5-rail@3.1.1",
    )


def test_known_failed_terminal_does_not_fence_later_commands() -> None:
    """MoveIt 返回失败终态后应保持 failed，刷新后仍可继续调试。"""

    port = _ScriptedJointPort(
        [
            {
                "state": "failed",
                "completed": False,
                "message": "MoveIt 返回失败终态",
            },
            {
                "state": "succeeded",
                "completed": True,
                "message": "MoveIt 完成",
            },
        ]
    )
    adapter = _adapter(port)

    failed = adapter.execute_commissioning(_move_target("cmd-fail"))
    assert failed.state is CommandState.FAILED
    assert adapter.commissioning_snapshot().execution_fenced is False

    succeeded = adapter.execute_commissioning(_move_target("cmd-retry"))
    assert succeeded.state is CommandState.SUCCEEDED
    assert port.calls == 2


def test_ambiguous_exception_after_dispatch_keeps_fence() -> None:
    """派发后通信歧义仍必须 UNKNOWN 并保留 Fence，禁止自动重放。"""

    port = _ScriptedJointPort([RuntimeError("MoveIt action 传输中断")])
    adapter = _adapter(port)

    unknown = adapter.execute_commissioning(_move_target("cmd-unknown"))
    assert unknown.state is CommandState.EXECUTION_UNKNOWN
    assert adapter.commissioning_snapshot().execution_fenced is True

    blocked = adapter.execute_commissioning(_move_target("cmd-next"))
    assert blocked.state is CommandState.REJECTED
    assert "Fence" in blocked.message
    assert port.calls == 1
