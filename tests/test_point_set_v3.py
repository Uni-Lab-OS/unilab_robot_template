"""PointSet v3、阵列、导轨分配和 D9-1S 接近运动块合同测试。"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import pytest
from unilab_robot_contracts import (
    CartesianPose,
    InstallationCalibration,
    JointSpecification,
    JointType,
    ResolvedCartesianTarget,
    ResolvedJointTarget,
    RigidTransform,
    RobotPointSetResolver,
    ToolContext,
    patch_authored_joint_target_yaml,
    revise_authored_joint_target,
)


class _ArmModel:
    """使用一旋转一平移关节验证型号拥有顺序、单位和 FK。"""

    model_ref = "test:arm@1"
    base_frame = "arm_base"
    joint_specs = (
        JointSpecification("turn", JointType.REVOLUTE, -math.pi, math.pi),
        JointSpecification("slide", JointType.PRISMATIC, 0.0, 1.0),
    )

    def forward_kinematics(
        self,
        joint_positions: Sequence[float],
        tool_context: ToolContext,
    ) -> CartesianPose:
        """把测试关节转换为基座中的 TCP 位姿。

        参数：关节数组依次使用 rad、m；工具上下文提供固定 TCP。返回：
        基座坐标中的 TCP。异常：关节数量错误时抛出 ``ValueError``。
        """

        del tool_context
        angle, slide = tuple(float(value) for value in joint_positions)
        return CartesianPose(
            "arm_base",
            (slide, 0.0, 0.0),
            (0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0)),
        )


@dataclass(frozen=True)
class _RailModel:
    """测试使用的两米单轴导轨型号。"""

    model_ref: str = "test:rail@1"
    travel_m: tuple[float, float] = (0.0, 2.0)


def _tool() -> ToolContext:
    """返回解析测试冻结的工具上下文。"""

    return ToolContext(
        "test-tool@1",
        "1" * 64,
        RigidTransform.identity(),
        1,
    )


def _calibration() -> InstallationCalibration:
    """返回把货架局部坐标平移到机械臂基座的安装标定。"""

    return InstallationCalibration(
        "test-installation@1.0.0",
        "2" * 64,
        {
            "device:deck.group-rack": RigidTransform(
                (0.1, 0.2, 0.3),
                (0.0, 0.0, 0.0, 1.0),
            )
        },
    )


def _base_point_set() -> dict[str, object]:
    """返回含全局避障点和 4×3 阵列的最小 v3 作者资产。"""

    return {
        "schema": "unilab.robot-point-set/v3",
        "revision": "test-points@3.0.0",
        "components": {
            "arm": {
                "model_ref": "test:arm@1",
                "tool_context_ref": "test-tool@1",
            },
            "rail": {"model_ref": "test:rail@1"},
        },
        "installation_calibration": {
            "revision": "test-installation@1.0.0",
            "digest": "2" * 64,
        },
        "global": {
            "arm": {
                "standby": {
                    "type": "joint_positions",
                    "value": [0.0, 0.2],
                    "source_point": "P-standby",
                },
                "rail_transfer_safe": {
                    "type": "joint_positions",
                    "value": [0.1, 0.1],
                },
            },
            "rail": {"home": {"position_si": 0.0}},
        },
        "group-rack": {
            "device_ref": "deck.group-rack",
            "display_name": "组合货架",
            "access": {
                "type": "access_motion_block/v1",
                "frame_ref": "device:deck.group-rack",
                "entry_offset_xyz_m": [0.15, 0.0, 0.10],
                "approach_offset_xyz_m": [0.0, 0.0, 0.10],
                "orientation_policy": "inherit_interaction",
                "transit_in": ["global.arm.standby"],
                "transit_out": {"mode": "reverse_transit_in"},
            },
            "grid": {
                "type": "affine_grid/v1",
                "rows": 4,
                "cols": 3,
                "frame_ref": "device:deck.group-rack",
                "anchors": {
                    "r1c1": {"xyz_m": [0.0, 0.0, 0.0]},
                    "r4c1": {"xyz_m": [0.3, 0.0, 0.0]},
                    "r4c3": {
                        "xyz_m": [0.3, 0.2, 0.0],
                        "orientation_override_xyzw": [0.0, 0.0, 1.0, 0.0],
                    },
                },
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                "corrections": {
                    "r2c2": {
                        "xyz_m": [0.01, -0.01, 0.02],
                        "orientation_override_xyzw": [0.0, 0.0, 1.0, 0.0],
                    }
                },
                "rail": {
                    "default": 0.45,
                    "groups": [
                        {
                            "position": 0.30,
                            "cells": ["r1c1", "r2c1", "r3c1", "r4c1"],
                        },
                        {
                            "position": 0.60,
                            "cells": ["r1c3", "r2c3", "r3c3", "r4c3"],
                        },
                    ],
                    "overrides": {"r4c3": 0.52},
                },
            },
        },
    }


def _resolver(point_set: dict[str, object] | None = None) -> RobotPointSetResolver:
    """用固定型号、工具、标定和导轨解析一个 v3 资产。"""

    return RobotPointSetResolver(
        point_set or _base_point_set(),
        arm_model=_ArmModel(),
        tool_context=_tool(),
        calibration=_calibration(),
        rail_model=_RailModel(),
    )


def test_resolve_arm_target_accepts_authored_source_point() -> None:
    """作者层 source_point 必须由 PointSet 解析器解析，不能在站点代码写死别名。"""

    resolver = _resolver()
    by_ref = resolver.resolve_arm_target("global.arm.standby")
    by_source = resolver.resolve_arm_target("P-standby")

    assert isinstance(by_ref, ResolvedJointTarget)
    assert by_source.target_ref == "global.arm.standby"
    assert by_source.joint_positions == by_ref.joint_positions
    with pytest.raises(ValueError, match="缺少 arm target_ref"):
        resolver.resolve_arm_target("missing-point")


def test_affine_grid_uses_three_anchors_sparse_correction_and_rail_precedence() -> None:
    """4×3 阵列应按 D6/D6d 生成，并执行 override>group>default。"""

    resolver = _resolver()
    targets = resolver.resolve_all()

    assert len(targets) == 12
    assert targets["group-rack.r1c1"].rail_position_si == pytest.approx(0.30)
    assert targets["group-rack.r2c2"].rail_position_si == pytest.approx(0.45)
    assert targets["group-rack.r4c3"].rail_position_si == pytest.approx(0.52)
    anchor = resolver.resolve_arm_target("group-rack.r4c3.interaction")
    assert isinstance(anchor, ResolvedCartesianTarget)
    assert anchor.pose.orientation_xyzw == pytest.approx((0.0, 0.0, 1.0, 0.0))
    corrected = resolver.resolve_arm_target("group-rack.r2c2.interaction")
    assert isinstance(corrected, ResolvedCartesianTarget)
    assert corrected.pose.xyz_m == pytest.approx((0.21, 0.29, 0.32))
    assert corrected.pose.orientation_xyzw == pytest.approx((0.0, 0.0, 1.0, 0.0))
    assert len(targets["group-rack.r2c2"].resolved_digest) == 64


def test_access_block_offsets_share_one_interaction_anchor_and_reverse_transit() -> None:
    """D9-1S 的 entry/approach 必须直接相对同一交互锚点并整体反向退出。"""

    resolver = _resolver()
    target = resolver.resolve("group-rack.r1c1")
    block = target.access_block
    assert block is not None
    assert block.transit_in == ("global.arm.standby",)
    assert block.transit_out == ("global.arm.standby",)
    interaction = resolver.resolve_arm_target(block.interaction_target_ref)
    approach = resolver.resolve_arm_target(block.approach_target_ref)
    entry = resolver.resolve_arm_target(block.entry_target_ref)
    assert isinstance(interaction, ResolvedCartesianTarget)
    assert isinstance(approach, ResolvedCartesianTarget)
    assert isinstance(entry, ResolvedCartesianTarget)
    assert interaction.pose.xyz_m == pytest.approx((0.1, 0.2, 0.3))
    assert approach.pose.xyz_m == pytest.approx((0.1, 0.2, 0.4))
    assert entry.pose.xyz_m == pytest.approx((0.25, 0.2, 0.4))
    assert approach.source_chain[-2:] == (
        "group-rack.r1c1.interaction",
        "group-rack.r1c1.approach",
    )
    assert entry.source_chain[-2:] == (
        "group-rack.r1c1.interaction",
        "group-rack.r1c1.entry",
    )


def test_access_block_converts_joint_interaction_to_absolute_tcp() -> None:
    """显式关节交互点应先经 FK 固化为 TCP，再应用 D9-1S 偏移。"""

    point_set = _base_point_set()
    point_set.pop("group-rack")
    point_set["fixture"] = {
        "device_ref": "deck.fixture",
        "transit": {
            "ready": {"type": "joint_positions", "value": [0.0, 0.1]}
        },
        "targets": {
            "slot-1": {
                "rail": {"position_si": 0.4},
                "arm": {
                    "type": "joint_positions",
                    "value": [math.pi / 2.0, 0.2],
                },
                "access": {
                    "type": "access_motion_block/v1",
                    "frame_ref": "arm_base",
                    "entry_offset_xyz_m": [0.1, 0.0, 0.1],
                    "approach_offset_xyz_m": [0.0, 0.0, 0.1],
                    "transit_in": ["fixture.transit.ready"],
                },
            }
        },
    }

    resolver = _resolver(point_set)
    interaction = resolver.resolve_arm_target("fixture.slot-1.interaction")
    assert isinstance(interaction, ResolvedCartesianTarget)
    assert interaction.pose.xyz_m == pytest.approx((0.2, 0.0, 0.0))
    assert interaction.ik_seed == pytest.approx((math.pi / 2.0, 0.2))
    assert resolver.resolve("fixture.slot-1").access_block is not None


def test_single_arm_v3_omits_rail_without_second_schema() -> None:
    """单机械臂继续使用 v3，且完全省略 rail component 和 rail 字段。"""

    point_set = _base_point_set()
    components = point_set["components"]
    assert isinstance(components, dict)
    components.pop("rail")
    grid_group = point_set.pop("group-rack")
    assert isinstance(grid_group, dict)
    point_set["tool-change"] = {
        "device_ref": "deck.tool-change",
        "targets": {
            "ready": {
                "arm": {
                    "type": "cartesian_pose",
                    "frame_ref": "device:deck.group-rack",
                    "value": {
                        "xyz_m": [0.2, 0.1, 0.3],
                        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                }
            }
        },
    }
    global_targets = point_set["global"]
    assert isinstance(global_targets, dict)
    global_targets.pop("rail")

    resolver = RobotPointSetResolver(
        point_set,
        arm_model=_ArmModel(),
        tool_context=_tool(),
        calibration=_calibration(),
    )

    assert resolver.resolve("tool-change.ready").rail_position_si is None
    assert resolver.rail_targets == {}


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (
            lambda value: value.update({"schema": "unilab.robot-point-set/v2"}),
            "v3",
        ),
        (
            lambda value: value["group-rack"]["grid"]["corrections"].update(
                {"r1c1": {"xyz_m": [0.0, 0.0, 0.0]}}
            ),
            "锚点",
        ),
        (
            lambda value: value["group-rack"]["grid"]["rail"]["groups"].append(
                {"position": 0.7, "cells": ["r1c1"]}
            ),
            "重复覆盖",
        ),
        (
            lambda value: value["group-rack"]["grid"]["corrections"].update(
                {"r9c9": {"xyz_m": [0.0, 0.0, 0.0]}}
            ),
            "越界",
        ),
    ],
)
def test_invalid_v3_assets_fail_closed(mutate: object, message: str) -> None:
    """旧 Schema、双真值锚点、重叠分组和越界 cell 必须在激活前拒绝。"""

    point_set = _base_point_set()
    mutate(point_set)  # type: ignore[operator]
    with pytest.raises(ValueError, match=message):
        _resolver(point_set)


def _fixture_point_set() -> dict[str, object]:
    """返回含 transit 与带 access 的显式关节交互点的作者资产。"""

    point_set = _base_point_set()
    point_set.pop("group-rack")
    point_set["fixture"] = {
        "device_ref": "deck.fixture",
        "transit": {
            "ready": {"type": "joint_positions", "value": [0.0, 0.1]}
        },
        "targets": {
            "slot-1": {
                "rail": {"position_si": 0.4},
                "arm": {
                    "type": "joint_positions",
                    "value": [math.pi / 2.0, 0.2],
                    "source_point": "P-slot",
                },
                "access": {
                    "type": "access_motion_block/v1",
                    "frame_ref": "arm_base",
                    "entry_offset_xyz_m": [0.1, 0.0, 0.1],
                    "approach_offset_xyz_m": [0.0, 0.0, 0.1],
                    "transit_in": ["fixture.transit.ready"],
                },
            }
        },
    }
    return point_set


def test_authoring_catalog_lists_joint_positions_and_omits_access_offsets() -> None:
    """调试目录只暴露作者层关节目标，不把 entry/approach 或阵列展开成可写点。"""

    catalog = _resolver().authoring_catalog()
    refs = tuple(entry.target_ref for entry in catalog)

    assert refs == ("global.arm.rail_transfer_safe", "global.arm.standby")
    standby = next(entry for entry in catalog if entry.target_ref == "global.arm.standby")
    assert standby.source_point == "P-standby"
    assert standby.editable is True
    assert standby.kind == "joint_positions"
    assert standby.joint_positions_si == (0.0, 0.2)
    assert standby.group_ref == "global.arm"


def test_authoring_catalog_includes_interaction_seed_and_rail() -> None:
    """带 AccessMotionBlock 的显式关节点应以 interaction_seed 进入目录。"""

    catalog = _resolver(_fixture_point_set()).authoring_catalog()
    seed = next(
        entry
        for entry in catalog
        if entry.target_ref == "fixture.slot-1.interaction_seed"
    )

    assert seed.source_point == "P-slot"
    assert seed.rail_position_si == pytest.approx(0.4)
    assert seed.joint_positions_si == pytest.approx((math.pi / 2.0, 0.2))
    assert "fixture.slot-1.entry" not in {entry.target_ref for entry in catalog}
    assert "fixture.slot-1.approach" not in {entry.target_ref for entry in catalog}


def test_revise_authored_joint_target_updates_seed_and_bumps_revision() -> None:
    """示教写回只改作者层 value，并提升 PointSet revision。"""

    original = _fixture_point_set()
    revised = revise_authored_joint_target(
        original,
        "fixture.slot-1.interaction_seed",
        (0.3, 0.4),
        joint_count=2,
    )

    assert original["revision"] == "test-points@3.0.0"
    assert revised["revision"] == "test-points@3.0.1"
    assert revised["fixture"]["targets"]["slot-1"]["arm"]["value"] == [0.3, 0.4]
    seed = _resolver(revised).resolve_arm_target("fixture.slot-1.interaction_seed")
    assert isinstance(seed, ResolvedJointTarget)
    assert seed.joint_positions == pytest.approx((0.3, 0.4))


def test_patch_authored_joint_target_yaml_rewrites_only_revision_and_that_value() -> None:
    """示教写回只替换 revision 标量和目标关节列表，其它注释与点位原文不变。"""

    original = """schema: unilab.robot-point-set/v3
revision: demo@1.2.3
# keep this author comment
description: untouched prose
global:
  arm:
    home:
      type: joint_positions
      value:
      - 0.100000000000
      - 0.200000000000
      source_point: P1
    standby:
      type: joint_positions
      value:
      - 0.300000000000
      - 0.400000000000
      source_point: P2
"""
    patched = patch_authored_joint_target_yaml(
        original,
        "global.arm.home",
        (0.5, 0.6),
        joint_count=2,
    )

    assert "# keep this author comment" in patched
    assert "description: untouched prose" in patched
    assert "revision: demo@1.2.4" in patched
    assert "      - 0.300000000000\n      - 0.400000000000\n      source_point: P2" in patched
    assert original.split("standby:")[1] == patched.split("standby:")[1]
    home_value = patched.split("home:")[1].split("standby:")[0]
    assert "- 0.5\n" in home_value or "- 0.500000000000" in home_value
    assert "- 0.6\n" in home_value or "- 0.600000000000" in home_value
    assert "- 0.100000000000" not in home_value


def test_revise_authored_joint_target_rejects_access_offsets_and_wrong_count() -> None:
    """派生接近点和关节数量错误不得写回 PointSet。"""

    point_set = _fixture_point_set()
    with pytest.raises(ValueError, match="没有可写回"):
        revise_authored_joint_target(
            point_set,
            "fixture.slot-1.entry",
            (0.3, 0.4),
            joint_count=2,
        )
    with pytest.raises(ValueError, match="关节数量"):
        revise_authored_joint_target(
            point_set,
            "global.arm.standby",
            (0.1,),
            joint_count=2,
        )

