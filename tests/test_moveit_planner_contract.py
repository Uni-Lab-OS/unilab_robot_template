"""MoveItPlanner lazy-import, fail-closed and scene-validation tests."""

from __future__ import annotations

import importlib.util
import math

import pytest
from unilab_robot_contracts import (
    CartesianGoal,
    ClearanceConstraint,
    CollisionGeometry,
    CollisionObject,
    CollisionWorld,
    PlanConstraints,
    PlanGoal,
    PlanRequest,
    RobotModel,
)
from unilab_robot_runtime.moveit_planner import (
    MoveItPlanner,
    MoveItPlannerConfig,
    PlannerUnavailableError,
    detect_planner_identity,
)


def _model() -> RobotModel:
    urdf = (
        '<robot name="one_axis">'
        '<link name="base"/><link name="tool"/>'
        '<joint name="j1" type="revolute">'
        '<parent link="base"/><child link="tool"/>'
        '<limit lower="-1" upper="1" effort="1" velocity="1"/>'
        "</joint></robot>"
    )
    srdf = (
        '<robot name="one_axis"><group name="arm">'
        '<joint name="j1"/></group></robot>'
    )
    return RobotModel(
        "one-axis",
        urdf,
        srdf,
        "base",
        "tool",
        "arm",
        ("j1",),
        ((-1.0, 1.0),),
    )


def _request(*, clearance: bool = False) -> PlanRequest:
    return PlanRequest(
        _model(),
        (0.0,),
        PlanGoal.from_pose(
            CartesianGoal("world", (0.1, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0))
        ),
        collision_world=CollisionWorld(),
        constraints=PlanConstraints(
            clearance=ClearanceConstraint(0.01) if clearance else None
        ),
        timeout_s=1.0,
    )


def test_identity_is_read_without_importing_moveit() -> None:
    """规划器身份必须是显式探测值，允许环境未安装时为 unknown。"""

    identity = detect_planner_identity()
    assert identity.name == "MoveItPy"
    assert identity.implementation == "moveit_py_fcl"
    assert set(identity.engine_versions) == {"moveit", "moveit_msgs", "fcl", "ompl"}
    assert all(isinstance(value, str) and value for value in identity.engine_versions.values())
    planner = MoveItPlanner(MoveItPlannerConfig(planning_time_s=1.0))
    assert planner.identity.name == "MoveItPy"


def test_clearance_is_rejected_before_any_native_import() -> None:
    """未原生实现的 clearances 必须 fail closed，而不是静默忽略。"""

    planner = MoveItPlanner()
    result = planner.plan(_request(clearance=True))
    assert result.status == "invalid"
    assert result.waypoints == ()
    assert result.diagnostics["failure_category"] == (
        "clearance_constraint_not_supported"
    )
    planner.close()


@pytest.mark.skipif(
    importlib.util.find_spec("moveit") is not None,
    reason="this test asserts the no-MoveIt failure path",
)
def test_missing_moveit_returns_typed_result_and_preserves_cause() -> None:
    """无 MoveIt 时返回 invalid，不把 ImportError 泄漏给调用方。"""

    planner = MoveItPlanner()
    result = planner.plan(_request())
    assert result.status == "invalid"
    assert result.waypoints == ()
    assert result.diagnostics["failure_category"] == "planner_unavailable"
    planner.close()


def test_scene_validation_is_independent_of_moveit() -> None:
    """场景校验只检查值对象和文件存在，不启动 MoveIt。"""

    scene = CollisionWorld(
        objects=(
            CollisionObject(
                "missing",
                (
                    (1.0, 0.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0, 0.0),
                    (0.0, 0.0, 1.0, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                ),
                (CollisionGeometry("mesh", mesh_path="/definitely/missing.stl"),),
            ),
        )
    )
    report = MoveItPlanner().validate(scene)
    assert report["status"] == "invalid"
    assert report["collision_query"] == "not_run"
    assert report["issues"][0]["kind"] == "missing_mesh"


def test_config_rejects_nonpositive_time_and_bad_sample_step() -> None:
    """配置错误在构造期失败，不等到原生规划。"""

    with pytest.raises(ValueError):
        MoveItPlannerConfig(planning_time_s=0.0)
    with pytest.raises(ValueError):
        MoveItPlannerConfig(max_joint_sample_step_rad=float("nan"))
    with pytest.raises(ValueError):
        MoveItPlannerConfig(planning_attempts=0)


def test_unavailable_error_keeps_original_import_error() -> None:
    """保留原始错误链是排查环境差异的要求。"""

    planner = MoveItPlanner()
    if importlib.util.find_spec("moveit") is not None:
        pytest.skip("native MoveIt present")
    with pytest.raises(PlannerUnavailableError) as captured:
        planner._require_native()
    assert isinstance(captured.value.__cause__, ImportError)


def test_math_import_is_used_for_config_only() -> None:
    """防止坐标约定常量被误改成角度制。"""

    config = MoveItPlannerConfig()
    assert math.isfinite(config.velocity_scaling)
    assert config.planner_id == "RRTConnectkConfigDefault"
