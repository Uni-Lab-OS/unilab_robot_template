"""机械臂 typed point-set 与相对目标解析的公开合同。"""

from __future__ import annotations

import math
from collections.abc import Sequence

import pytest
from unilab_robot_contracts import (
    CartesianPose,
    JointSpecification,
    JointType,
    MotionTargetResolver,
    ResolvedCartesianTarget,
    ResolvedJointTarget,
    RigidTransform,
    ToolContext,
)


class _MixedJointModel:
    """用一旋转一平移关节证明按关节类型解释 SI 值。"""

    model_ref = "test:mixed-joint-model@1"
    base_frame = "arm_base"
    tip_frame = "tool_mount"
    joint_specs = (
        JointSpecification("turn", JointType.REVOLUTE, -math.pi, math.pi),
        JointSpecification("slide", JointType.PRISMATIC, 0.0, 1.0),
    )

    def forward_kinematics(
        self,
        joint_positions: Sequence[float],
        tool_context: ToolContext,
    ) -> CartesianPose:
        """将测试关节值转换为绝对 TCP 位姿。

        参数：``joint_positions`` 依次是弧度和米；``tool_context`` 固定 TCP。
        返回：基座坐标系中的 TCP 位姿。
        异常：关节数量错误时抛出 ``ValueError``。
        """

        if len(tuple(joint_positions)) != 2:
            raise ValueError("测试模型必须接收两个关节")
        angle, slide = (float(value) for value in joint_positions)
        flange = RigidTransform(
            (slide, 0.0, 0.0),
            (0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0)),
        )
        tcp = flange.compose(tool_context.mount_to_tcp)
        return CartesianPose(self.base_frame, tcp.translation_m, tcp.orientation_xyzw)


def _tool_context() -> ToolContext:
    """返回测试使用的固定 TCP 上下文。"""

    return ToolContext(
        context_id="test-tool@1",
        digest="0" * 64,
        mount_to_tcp=RigidTransform.identity(),
        attachment_generation=1,
    )


def _point_set(targets: dict[str, object]) -> dict[str, object]:
    """把 waypoint 映射包装成最小合法 v2 点位资产。"""

    return {
        "schema": "unilab.arm-point-set/v2",
        "revision": "test-points@1.0.0",
        "compatible_model_ref": _MixedJointModel.model_ref,
        "tool_context_ref": "test-tool@1",
        "targets": {
            "position1": {
                "description": "测试位置",
                "waypoints": targets,
            }
        },
    }


def test_joint_reference_uses_fk_and_preserves_joint_seed() -> None:
    """相对关节点必须先做 FK，并保留原关节值作为 IK 构型种子。"""

    resolver = MotionTargetResolver(
        _point_set(
            {
                "approach": {
                    "type": "joint_positions",
                    "value": [math.pi / 2.0, 0.2],
                },
                "interaction": {
                    "type": "cartesian_delta",
                    "relative_to": "position1.approach",
                    "frame_ref": "reference_target",
                    "value": {
                        "xyz_m": [0.1, 0.0, 0.0],
                        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                },
            }
        ),
        model=_MixedJointModel(),
        tool_context=_tool_context(),
    )

    anchor = resolver.resolve("position1.approach")
    resolved = resolver.resolve("position1.interaction")

    assert isinstance(anchor, ResolvedJointTarget)
    assert anchor.joint_positions == pytest.approx((math.pi / 2.0, 0.2))
    assert isinstance(resolved, ResolvedCartesianTarget)
    assert resolved.pose.xyz_m == pytest.approx((0.2, 0.1, 0.0))
    assert resolved.ik_seed == pytest.approx(anchor.joint_positions)
    assert resolved.source_chain == (
        "position1.approach",
        "position1.interaction",
    )


def test_cartesian_delta_chain_resolves_from_absolute_anchor() -> None:
    """多级相对点必须按依赖顺序展开为一个绝对笛卡尔目标。"""

    resolver = MotionTargetResolver(
        _point_set(
            {
                "origin": {
                    "type": "cartesian_pose",
                    "frame_ref": "arm_base",
                    "value": {
                        "xyz_m": [0.4, 0.1, 0.25],
                        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                },
                "approach": {
                    "type": "cartesian_delta",
                    "relative_to": "position1.origin",
                    "frame_ref": "arm_base",
                    "value": {
                        "xyz_m": [0.0, 0.0, 0.1],
                        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                },
                "interaction": {
                    "type": "cartesian_delta",
                    "relative_to": "position1.approach",
                    "frame_ref": "reference_target",
                    "value": {
                        "xyz_m": [0.0, 0.0, -0.05],
                        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                },
            }
        ),
        model=_MixedJointModel(),
        tool_context=_tool_context(),
    )

    resolved = resolver.resolve("position1.interaction")

    assert isinstance(resolved, ResolvedCartesianTarget)
    assert resolved.pose.xyz_m == pytest.approx((0.4, 0.1, 0.3))
    assert resolved.source_chain == (
        "position1.origin",
        "position1.approach",
        "position1.interaction",
    )


@pytest.mark.parametrize(
    ("point_set", "message"),
    [
        (
            {
                **_point_set(
                    {
                        "approach": {
                            "type": "joint_positions",
                            "value": [0.0, 0.2],
                        }
                    }
                ),
                "joint_names": ["turn", "slide"],
            },
            "joint_names",
        ),
        (
            _point_set(
                {
                    "approach": {
                        "type": "joint_positions",
                        "unit": "rad",
                        "value": [0.0, 0.2],
                    }
                }
            ),
            "unit",
        ),
    ],
)
def test_v2_rejects_duplicate_joint_metadata(
    point_set: dict[str, object],
    message: str,
) -> None:
    """v2 点位资产不得重复型号描述符拥有的关节名或单一单位。"""

    with pytest.raises(ValueError, match=message):
        MotionTargetResolver(
            point_set,
            model=_MixedJointModel(),
            tool_context=_tool_context(),
        )


def test_delta_cycle_and_missing_anchor_fail_before_dispatch() -> None:
    """相对引用成环或缺少绝对锚点时必须在派发前关闭失败。"""

    resolver = MotionTargetResolver(
        _point_set(
            {
                "a": {
                    "type": "cartesian_delta",
                    "relative_to": "position1.b",
                    "frame_ref": "arm_base",
                    "value": {
                        "xyz_m": [0.0, 0.0, 0.1],
                        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                },
                "b": {
                    "type": "cartesian_delta",
                    "relative_to": "position1.a",
                    "frame_ref": "arm_base",
                    "value": {
                        "xyz_m": [0.0, 0.0, 0.1],
                        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                },
            }
        ),
        model=_MixedJointModel(),
        tool_context=_tool_context(),
    )

    with pytest.raises(ValueError, match="循环"):
        resolver.resolve("position1.a")


def test_tool_context_and_joint_limits_are_validated() -> None:
    """工具代次不匹配或关节越界时不得产生可执行目标。"""

    wrong_tool = ToolContext(
        context_id="other-tool@1",
        digest="1" * 64,
        mount_to_tcp=RigidTransform.identity(),
        attachment_generation=1,
    )
    with pytest.raises(ValueError, match="tool_context_ref"):
        MotionTargetResolver(
            _point_set({}),
            model=_MixedJointModel(),
            tool_context=wrong_tool,
        )

    resolver = MotionTargetResolver(
        _point_set(
            {
                "bad": {
                    "type": "joint_positions",
                    "value": [0.0, 1.2],
                }
            }
        ),
        model=_MixedJointModel(),
        tool_context=_tool_context(),
    )
    with pytest.raises(ValueError, match="slide"):
        resolver.resolve("position1.bad")
