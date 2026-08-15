"""MoveIt 调试 Adapter 必须区分已知失败与派发不明 Fence。"""

from __future__ import annotations

import time
from typing import Any

from unilab_arm_cr5 import MODEL_DESCRIPTOR
from unilab_arm_cr5.adapters import MoveItCommissioningAdapter
from unilab_robot_contracts import (
    CommandState,
    MoveTargetCommand,
    ResolvedJointTarget,
)


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
