"""快换 ToolContext 对 MoveIt TCP 目标与碰撞模型的合同测试。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

import pytest
from unilab_arm_cr7.adapters import MoveIt2ClientPort
from unilab_robot_contracts import RigidTransform, ToolContext


class _Client:
    """记录法兰目标并确认 PlanningScene 工具应用的无 ROS 替身。"""

    def __init__(self) -> None:
        """创建空记录与 MoveIt 缩放字段。"""

        self.max_velocity = 1.0
        self.max_acceleration = 1.0
        self.pose: tuple[float, ...] | None = None
        self.collision_asset_resolver: Any = None

    def apply_tool_context(
        self,
        context: ToolContext,
        *,
        collision_asset_resolver: Any = None,
    ) -> Mapping[str, Any]:
        """返回精确绑定摘要与附着代次的确认。"""

        self.collision_asset_resolver = collision_asset_resolver
        return {
            "applied": True,
            "tool_context_digest": context.digest,
            "attachment_generation": context.attachment_generation,
        }

    def move_to_pose(
        self,
        *,
        position: Sequence[float],
        quat_xyzw: Sequence[float],
        **kwargs: Any,
    ) -> None:
        """记录 Adapter 换算后的法兰位姿。"""

        del kwargs
        self.pose = (*map(float, position), *map(float, quat_xyzw))

    def wait_until_executed(self) -> bool:
        """确认测试运动完成。"""

        return True


def _tool_context() -> ToolContext:
    """返回 TCP 比法兰沿 Z 轴前伸 0.1 m 的测试工具。"""

    return ToolContext(
        "quick-gripper@1",
        "a" * 64,
        RigidTransform((0.0, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0)),
        3,
        {
            "attached_body_id": "quick-gripper",
            "parent_link": "cr7_link_6",
            "collision_primitives": [
                {
                    "shape": "box",
                    "size_m": [0.1, 0.1, 0.2],
                    "pose": {
                        "xyz_m": [0.0, 0.0, 0.1],
                        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                }
            ],
        },
    )


def test_moveit_converts_desired_tcp_pose_to_mount_pose_after_tool_activation() -> None:
    """目标 TCP 为 z=0.5 m 时，法兰应因工具前伸而规划到 z=0.4 m。"""

    client = _Client()
    port = MoveIt2ClientPort(
        client,
        qualified_joint_names=tuple(
            f"robot_cr7_joint_{index}" for index in range(1, 7)
        ),
    )
    receipt = port.apply_tool_context(_tool_context())

    result = port.execute_cartesian_target(
        group_name="cr7_arm",
        frame_ref="arm_base",
        xyz_m=(0.2, 0.1, 0.5),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        command_id="tcp-target-1",
        parameters={},
    )

    assert receipt["applied"] is True
    assert result["completed"] is True
    assert client.pose == pytest.approx((0.2, 0.1, 0.4, 0.0, 0.0, 0.0, 1.0))


def test_moveit_refuses_tool_without_planning_scene_geometry() -> None:
    """只有 TCP 而没有碰撞几何的快换工具不得进入 MoveIt 执行面。"""

    port = MoveIt2ClientPort(
        _Client(),
        qualified_joint_names=tuple(
            f"robot_cr7_joint_{index}" for index in range(1, 7)
        ),
    )
    incomplete = ToolContext(
        "incomplete@1",
        "b" * 64,
        RigidTransform.identity(),
        1,
    )

    with pytest.raises(ValueError, match="PlanningScene"):
        port.apply_tool_context(incomplete)


def test_moveit_passes_domain_asset_resolver_to_os_tool_scene() -> None:
    """Robotics 只转交受信资产解析器，不在通用包解释领域 URI。"""

    client = _Client()
    resolver = lambda reference: reference
    port = MoveIt2ClientPort(
        client,
        qualified_joint_names=tuple(
            f"robot_cr7_joint_{index}" for index in range(1, 7)
        ),
        collision_asset_resolver=resolver,
    )

    port.apply_tool_context(_tool_context())

    assert client.collision_asset_resolver is resolver
