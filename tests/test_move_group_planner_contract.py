"""``MoveGroupPlanner`` 的合同测试（不需要 ROS/MoveIt 运行时）。"""

from __future__ import annotations

import importlib.util

import pytest

from unilab_robot_contracts.planning import (
    ClearanceConstraint,
    CollisionGeometry,
    CollisionObject,
    CollisionWorld,
    PlanConstraints,
    PlanGoal,
    PlanRequest,
    RobotModel,
)
from unilab_robot_runtime.moveit_move_group import (
    MoveGroupPlanner,
    MoveGroupPlannerConfig,
    MoveGroupUnavailableError,
    build_move_group_parameters,
    detect_planner_identity,
)

URDF = """<?xml version="1.0"?><robot name="stub">
  <link name="world"/><link name="base_link"/><link name="link_1"/>
  <joint name="world_to_base" type="fixed"><origin xyz="0 0 0.1" rpy="0 0 0"/>
    <parent link="world"/><child link="base_link"/></joint>
  <joint name="joint_1" type="revolute"><origin xyz="0 0 0"/><parent link="base_link"/>
    <child link="link_1"/><axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="3.14"/></joint>
</robot>"""

SRDF = """<?xml version="1.0"?><robot name="stub">
  <group name="arm"><chain base_link="base_link" tip_link="link_1"/></group>
  <disable_collisions link1="base_link" link2="link_1" reason="Adjacent"/>
</robot>"""


def robot_model() -> RobotModel:
    return RobotModel(
        model_id="stub",
        urdf_xml=URDF,
        srdf_xml=SRDF,
        base_frame="world",
        flange_link="link_1",
        planning_group="arm",
        joint_names=("joint_1",),
        joint_limits=((-3.0, 3.0),),
    )


def request(**overrides) -> PlanRequest:
    payload = {
        "robot": robot_model(),
        "start_q": (0.0,),
        "goal": PlanGoal.from_joint([0.5]),
        "timeout_s": 1.0,
    }
    payload.update(overrides)
    return PlanRequest(**payload)


def test_config_rejects_invalid_values():
    with pytest.raises(ValueError):
        MoveGroupPlannerConfig(node_name="  ")
    with pytest.raises(ValueError):
        MoveGroupPlannerConfig(planning_time_s=0.0)
    with pytest.raises(ValueError):
        MoveGroupPlannerConfig(planning_attempts=0)
    with pytest.raises(ValueError):
        MoveGroupPlannerConfig(domain_id=-1)


def test_identity_is_detected_without_ros():
    identity = detect_planner_identity()
    assert identity.name == "MoveIt move_group"
    assert identity.implementation == "move_group_ros2_services"
    assert identity.engine_versions["transport"] == "ros2_services"
    assert MoveGroupPlanner().identity == identity


def test_parameters_use_singular_plugin_names_and_one_pipeline():
    config = MoveGroupPlannerConfig(planning_time_s=3.0)
    params = build_move_group_parameters(robot_model(), config)
    assert params["planning_pipelines"] == ["ompl"]
    assert params["default_planning_pipeline"] == "ompl"
    # MoveIt 2.5.9 先读单数 planning_plugin；只给复数会静默退回 CHOMP。
    assert params["ompl"]["planning_plugin"] == "ompl_interface/OMPLPlanner"
    assert params["ompl"]["planning_plugins"] == ["ompl_interface/OMPLPlanner"]
    assert "cartesian_limits" in params["robot_description_planning"]
    limits = params["robot_description_planning"]["joint_limits"]["joint_1"]
    assert limits["min_position"] == -3.0 and limits["max_position"] == 3.0
    assert limits["max_acceleration"] == config.joint_acceleration_rad_s2
    assert params["robot_description"] == URDF
    assert params["robot_description_semantic"] == SRDF
    assert params["plan_request_params"]["planning_time"] == 3.0


def test_missing_ros_returns_planner_unavailable(monkeypatch):
    planner = MoveGroupPlanner()

    def unavailable():
        raise MoveGroupUnavailableError("rclpy 不可用")

    monkeypatch.setattr(planner, "_import_native", unavailable)
    result = planner.plan(request())
    assert result.status == "invalid"
    assert result.diagnostics["failure_category"] == "planner_unavailable"
    assert result.waypoints == ()


def test_clearance_constraint_fails_closed_before_any_ros_call():
    planner = MoveGroupPlanner()
    constrained = request(constraints=PlanConstraints(
        clearance=ClearanceConstraint(min_clearance_m=0.01)))
    result = planner.plan(constrained)
    assert result.status == "invalid"
    assert result.diagnostics["failure_category"] == "clearance_constraint_not_supported"
    assert result.waypoints == ()


def test_zero_motion_is_detected_without_planning():
    from unilab_robot_runtime.moveit_move_group import _zero_motion

    assert _zero_motion(request(goal=PlanGoal.from_joint([0.0]))) is True
    assert _zero_motion(request(goal=PlanGoal.from_joint([0.5]))) is False


def test_validate_reports_geometry_without_claiming_a_collision_query():
    planner = MoveGroupPlanner()
    geometry = CollisionGeometry(kind="mesh",
                                 vertices=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                                 triangles=((0, 1, 2),))
    world = CollisionWorld(objects=(CollisionObject(
        object_id="table",
        world_pose=((1.0, 0.0, 0.0, 0.5), (0.0, 1.0, 0.0, 0.0),
                    (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        geometry=(geometry,)),))
    report = planner.validate(world)
    assert report["status"] == "valid"
    assert report["mesh_triangles"] == 1
    assert report["collision_query"] == "not_run"
    assert report["continuous_proof"] is False


def test_validate_flags_a_file_mesh_the_service_backend_cannot_send(tmp_path):
    planner = MoveGroupPlanner()
    geometry = CollisionGeometry(kind="mesh", mesh_path=str(tmp_path / "part.stl"))
    world = CollisionWorld(objects=(CollisionObject(
        object_id="part",
        world_pose=((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0),
                    (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        geometry=(geometry,)),))
    report = planner.validate(world)
    assert report["status"] == "invalid"
    assert report["issues"][0]["kind"] == "file_mesh_unsupported"


@pytest.mark.skipif(importlib.util.find_spec("rclpy") is not None,
                    reason="本用例只验证没有 rclpy 的解释器")
def test_plan_without_rclpy_is_typed_not_raised():
    result = MoveGroupPlanner().plan(request())
    assert result.status == "invalid"
    assert result.diagnostics["failure_category"] in {
        "planner_unavailable", "planner_dependency_missing"}

def test_rejected_native_path_is_diagnostic_and_checks_whole_robot(monkeypatch):
    from types import SimpleNamespace as S
    planner = MoveGroupPlanner()
    calls = []
    def validity(req, q, rail, *, whole_robot=False):
        calls.append((q, rail, whole_robot))
        return {"valid": q[0] < .2, "contact_details": [{"depth_m": .00001}]}
    monkeypatch.setattr(planner, "_state_validity", validity)
    native = S(joint_names=["joint_1"], points=[
        S(positions=[q], time_from_start=S(sec=0, nanosec=i * 100000000))
        for i, q in enumerate([0., .25, .5])])
    evidence = planner._rejected_trajectory(request(), native, [0.], 0.)
    assert evidence["executable"] is False
    assert len(evidence["points"]) == 3
    assert evidence["checked_states"] == 2
    assert evidence["first_invalid_state"]["index"] == 1
    assert all(row[2] for row in calls)
    assert evidence["first_invalid_state"]["validation"]["contact_details"][0]["depth_m"] == .00001


def test_pilz_terminal_stop_preserves_native_endpoint_and_duration():
    from unilab_robot_runtime.moveit_move_group import _coalesce_terminal_stop
    points = [
        dict(q=[0.], rail=r, velocities=(0.,), rail_velocity=v,
             accelerations=(0.,), rail_acceleration=a)
        for r,v,a in ((.5,0.,0.),(.4,-7.48e-16,.05),(.4,0.,0.))]
    times = [0.,10.199999998,10.199999998]
    receipt = _coalesce_terminal_stop(points,times)
    assert times == [0.,10.199999998]
    assert points[-1]["rail"] == .4 and points[-1]["rail_acceleration"] == 0.
    assert receipt["discarded"]["rail_acceleration"] == .05
    assert receipt["positions_unchanged"] and receipt["duration_unchanged"]


@pytest.mark.parametrize("change", [
    {"rail":.400001}, {"q":[.001]}, {"rail_velocity":.001},
    {"velocities":(.001,)}, {"rail_acceleration":.05}, {"accelerations":(.01,)}])
def test_duplicate_terminal_motion_or_nonstop_is_not_normalized(change):
    from copy import deepcopy
    from unilab_robot_runtime.moveit_move_group import _coalesce_terminal_stop
    last=dict(q=[0.],rail=.4,velocities=(0.,),rail_velocity=0.,
              accelerations=(0.,),rail_acceleration=0.)
    points=[dict(last,rail=.5),dict(last),dict(last,**change)]
    times=[0.,1.,1.]
    original=deepcopy(points)
    assert _coalesce_terminal_stop(points,times) is None
    assert points == original and times == [0.,1.,1.]


def test_interior_duplicate_and_missing_stop_derivatives_remain_invalid():
    from unilab_robot_runtime.moveit_move_group import _coalesce_terminal_stop
    point=dict(q=[0.],rail=.4,velocities=(0.,),rail_velocity=0.,
               accelerations=(0.,),rail_acceleration=0.)
    points=[dict(point) for _ in range(4)];times=[0.,.5,.5,1.]
    assert _coalesce_terminal_stop(points,times) is None
    assert times == [0.,.5,.5,1.]
    points=[dict(q=[0.],rail=.4) for _ in range(3)];times=[0.,1.,1.]
    assert _coalesce_terminal_stop(points,times) is None
    assert times == [0.,1.,1.]
