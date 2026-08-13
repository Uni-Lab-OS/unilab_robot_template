"""D4/D9-1S 的导轨 barrier、接近块与负载状态测试。"""

from __future__ import annotations

from unilab_robot_contracts import (
    AccessMotionBlock,
    AccessMotionPlanCompiler,
    ActionKind,
    MotionProfileCatalog,
    ResolvedPointTarget,
    WorkCellPhaseKind,
)


def _profiles() -> MotionProfileCatalog:
    """返回测试使用的机械臂、导轨和接近块策略。"""

    return MotionProfileCatalog.from_mapping(
        {
            "schema": "unilab.robot-motion-profiles/v1",
            "revision": "test-motion@1.0.0",
            "arm": {
                "transit": {
                    "primitive": "joint_ptp",
                    "velocity_scale": 0.25,
                    "acceleration_scale": 0.25,
                    "position_tolerance_m": 0.005,
                    "orientation_tolerance_rad": 0.02,
                },
                "linear": {
                    "primitive": "cartesian_linear",
                    "velocity_scale": 0.1,
                    "acceleration_scale": 0.1,
                    "position_tolerance_m": 0.002,
                    "orientation_tolerance_rad": 0.01,
                },
            },
            "rail": {
                "rail-safe": {
                    "velocity_scale": 0.2,
                    "acceleration_scale": 0.2,
                    "settle_tolerance_m": 0.001,
                }
            },
            "access": {
                "standard": {
                    "transit_profile_ref": "transit",
                    "entry_profile_ref": "linear",
                    "approach_profile_ref": "linear",
                    "interaction_profile_ref": "linear",
                    "rail_profile_ref": "rail-safe",
                }
            },
        }
    )


def _target(rail_position_si: float | None) -> ResolvedPointTarget:
    """构造一个已解析 D9-1S 目标。"""

    return ResolvedPointTarget(
        "rack.r1c1",
        "deck.rack",
        "rack.r1c1.interaction",
        rail_position_si,
        AccessMotionBlock(
            "rack.r1c1.access",
            "rack.r1c1.interaction",
            "rack.r1c1.approach",
            "rack.r1c1.entry",
            ("global.arm.standby",),
            ("global.arm.standby",),
            "reverse_transit_in",
        ),
        ("rack.r1c1",),
        "a" * 64,
    )


def test_rail_change_inserts_safe_pose_and_settle_before_arm_access() -> None:
    """导轨目标变化时必须先安全姿态、再导轨到位，最后才进入接近块。"""

    plan = AccessMotionPlanCompiler(profiles=_profiles()).compile(
        _target(0.5),
        action=ActionKind.PICK,
        policy_ref="standard",
        current_rail_position_si=0.3,
        has_rail=True,
    )

    assert [phase.phase_id for phase in plan.phases[:4]] == [
        "rail-transfer-safe",
        "rail-move-and-settle",
        "transit-in-1",
        "entry",
    ]
    assert plan.phases[1].kind is WorkCellPhaseKind.RAIL_MOVE
    assert all(phase.rail_position_si == 0.5 for phase in plan.phases)
    assert plan.phases[6].kind is WorkCellPhaseKind.END_EFFECTOR
    assert plan.phases[6].target_ref == "end_effector.grip"
    assert plan.phases[-1].payload_state == "loaded"


def test_missing_rail_holds_frozen_position_without_second_move() -> None:
    """点位省略 rail 时保持冻结位置，不能解释为导轨自由运动。"""

    plan = AccessMotionPlanCompiler(profiles=_profiles()).compile(
        _target(None),
        action=ActionKind.PLACE,
        policy_ref="standard",
        current_rail_position_si=0.45,
        has_rail=True,
    )

    assert plan.effective_rail_position_si == 0.45
    assert all(phase.kind is not WorkCellPhaseKind.RAIL_MOVE for phase in plan.phases)
    assert all(phase.rail_position_si == 0.45 for phase in plan.phases)
    assert any(phase.target_ref == "end_effector.release" for phase in plan.phases)


def test_single_arm_compiles_same_block_without_rail_phases() -> None:
    """单机械臂继续复用同一接近块，只省略全部导轨语义。"""

    plan = AccessMotionPlanCompiler(profiles=_profiles()).compile(
        _target(None),
        action=ActionKind.PICK,
        policy_ref="standard",
        current_rail_position_si=None,
        has_rail=False,
    )

    assert plan.effective_rail_position_si is None
    assert all(phase.kind is not WorkCellPhaseKind.RAIL_MOVE for phase in plan.phases)
    assert all(phase.rail_position_si is None for phase in plan.phases)
