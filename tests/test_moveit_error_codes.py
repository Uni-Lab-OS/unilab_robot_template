"""``MoveItErrorCodes`` 映射的回归测试（不需要 ROS/MoveIt 运行时）。

历史缺陷：``moveit_planner._status_for_error_code`` 把 -2/-16/-31 映射成
collision/timeout/unreachable，``moveit_move_group._ERROR_CODES`` 把 -26/-27/-28
记成 crash/abort/initialization、把 -31 记成 ``moveit_no_ik_solution`` 之外还
缺少 ``CRASH``/``ABORT``/``FAILURE``。这些码是判断"碰撞还是不可达"的唯一依据，
所以两条后端都必须查同一张表，且原始错误码/官方名称不能丢。
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from unilab_robot_contracts.planning import PlanGoal, PlanRequest, RobotModel
from unilab_robot_runtime import moveit_move_group, moveit_planner
from unilab_robot_runtime.moveit_error_codes import (
    MOVEIT_ERROR_CODES,
    UNREADABLE_MOVEIT_ERROR,
    UNRECOGNIZED_MOVEIT_ERROR,
    describe_moveit_error,
    error_code_fields,
    status_for_moveit_error,
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
        kinematics={"arm": {"kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin"}},
    )


def plan_request() -> PlanRequest:
    return PlanRequest(
        robot=robot_model(),
        start_q=(0.0,),
        goal=PlanGoal.from_joint([0.5]),
        timeout_s=1.0,
    )


# --------------------------------------------------------------------- 必需映射
@pytest.mark.parametrize(
    ("code", "status", "category", "name"),
    [
        (-2, "invalid", "invalid_motion_plan", "INVALID_MOTION_PLAN"),
        (-16, "invalid", "invalid_goal_constraints", "INVALID_GOAL_CONSTRAINTS"),
        (-31, "unreachable", "no_ik_solution", "NO_IK_SOLUTION"),
        (-1, "unreachable", "failure", "PLANNING_FAILED"),
    ],
)
def test_required_error_code_mapping(code, status, category, name):
    spec = describe_moveit_error(code)
    assert spec.code == code
    assert spec.status == status
    assert spec.category == category
    assert spec.name == name
    assert status_for_moveit_error(code) == status


def test_table_covers_every_code_of_moveit_2_5_9():
    # 逐条对照 Library/share/moveit_msgs/msg/MoveItErrorCodes.msg。
    expected = {
        1, 99999,
        -1, -2, -3, -4, -5, -6, -7,
        -10, -11, -12, -13, -14, -15, -16, -17, -18, -19,
        -21, -22, -23, -24, -25,
        -26, -27, -28, -29, -30, -31,
    }
    assert set(MOVEIT_ERROR_CODES) == expected


def test_codes_that_were_previously_misread():
    # -26/-27/-28 曾被记成 crash/abort/initialization；-29/-30 才是 CRASH/ABORT。
    assert MOVEIT_ERROR_CODES[-26].name == "START_STATE_INVALID"
    assert MOVEIT_ERROR_CODES[-27].name == "GOAL_STATE_INVALID"
    assert MOVEIT_ERROR_CODES[-28].name == "UNRECOGNIZED_GOAL_TYPE"
    assert MOVEIT_ERROR_CODES[-29].name == "CRASH"
    assert MOVEIT_ERROR_CODES[-30].name == "ABORT"
    assert MOVEIT_ERROR_CODES[-1].name == "PLANNING_FAILED"
    assert MOVEIT_ERROR_CODES[99999].name == "FAILURE"
    # -10/-12 才是碰撞起始/目标态，-2 不是。
    assert MOVEIT_ERROR_CODES[-10].status == "collision"
    assert MOVEIT_ERROR_CODES[-12].status == "collision"
    assert MOVEIT_ERROR_CODES[-2].status == "invalid"


def test_unknown_code_keeps_raw_value():
    spec = describe_moveit_error(424242)
    assert spec.code == 424242
    assert spec.name == UNRECOGNIZED_MOVEIT_ERROR
    assert spec.status == "invalid"
    assert spec.category == "moveit_error_424242"


@pytest.mark.parametrize("value", [None, "not-a-code", object(), True])
def test_unreadable_code_is_typed_not_raised(value):
    spec = describe_moveit_error(value)
    assert spec.code is None
    assert spec.name == UNREADABLE_MOVEIT_ERROR
    assert spec.status == "invalid"
    assert spec.category == "moveit_error_unreadable"


def test_error_code_fields_preserve_raw_code_and_name():
    fields = error_code_fields(-31)
    assert fields == {
        "moveit_error_code": -31,
        "moveit_error_name": "NO_IK_SOLUTION",
        "failure_category": "no_ik_solution",
    }


# --------------------------------------------------------------------- 两条后端
def test_planner_status_helper_agrees_with_the_table():
    for code, spec in MOVEIT_ERROR_CODES.items():
        assert moveit_planner._status_for_error_code(code) == spec.status
    assert moveit_planner._status_for_error_code(None) == "invalid"


def test_both_backends_share_one_table():
    assert moveit_move_group._ERROR_CODES is MOVEIT_ERROR_CODES


def test_move_group_native_segment_reports_category_and_official_name():
    class _Request:
        def __init__(self) -> None:
            self.motion_plan_request = SimpleNamespace(goal_constraints=[])

        @classmethod
        def Request(cls):
            return cls()

    response = SimpleNamespace(
        motion_plan_response=SimpleNamespace(
            error_code=SimpleNamespace(val=-31), trajectory=None, planning_time=0.01
        )
    )
    planner = moveit_move_group.MoveGroupPlanner()
    planner._session.messages = {"GetMotionPlan": SimpleNamespace(Request=_Request.Request)}
    planner._call = lambda *args, **kwargs: response
    planner._group = lambda request: "arm"
    planner._state = lambda request, q, rail=None: SimpleNamespace()
    planner._goal_constraints = lambda request, **kwargs: SimpleNamespace()

    segment = planner._native_segment(
        plan_request(),
        group="arm",
        start_q=(0.0,),
        joint_goal={"joint_1": 0.5},
        pose=None,
        rail_start=0.0,
        rail_goal=0.0,
    )
    assert segment["ok"] is False
    assert segment["status"] == "unreachable"
    assert segment["failure_category"] == "no_ik_solution"
    assert segment["error_code"] == -31
    assert segment["error_name"] == "NO_IK_SOLUTION"


def test_moveitpy_config_carries_the_singular_plugin_name():
    # MoveIt ≤2.5.x 只读单数 planning_plugin，缺省会静默退回 CHOMP。
    config = moveit_planner.MoveItPlanner()._build_config(plan_request())
    assert config["ompl"]["planning_plugin"] == "ompl_interface/OMPLPlanner"
    assert config["ompl"]["planning_plugins"] == ["ompl_interface/OMPLPlanner"]
    assert config["pilz"]["planning_plugin"] == "pilz_industrial_motion_planner/CommandPlanner"


@pytest.mark.parametrize(
    ("code", "status", "category", "name"),
    [
        (-2, "invalid", "invalid_motion_plan", "INVALID_MOTION_PLAN"),
        (-16, "invalid", "invalid_goal_constraints", "INVALID_GOAL_CONSTRAINTS"),
        (-31, "unreachable", "no_ik_solution", "NO_IK_SOLUTION"),
        (-1, "unreachable", "failure", "PLANNING_FAILED"),
    ],
)
def test_moveitpy_plan_preserves_raw_code_in_diagnostics(
    monkeypatch, code, status, category, name
):
    """两条后端都必须把原始码和官方名称带进 ``PlanResult.diagnostics``。"""
    planner = moveit_planner.MoveItPlanner(
        moveit_planner.MoveItPlannerConfig(planning_time_s=1.0)
    )
    monkeypatch.setattr(planner, "_ensure_robot", lambda request: object())
    monkeypatch.setattr(planner, "_load_world", lambda request: None)
    monkeypatch.setattr(planner, "_goal_state", lambda request, seed: (object(), None))
    monkeypatch.setattr(
        planner,
        "_native_segment",
        lambda *args, **kwargs: {
            "ok": False,
            "status": status,
            "failure_category": category,
            "error_code": code,
            "error_name": name,
            "error_message": "native failure",
            "planning_time_s": 0.01,
        },
    )
    result = planner.plan(plan_request())
    assert result.status == status
    assert result.waypoints == ()
    assert result.diagnostics["moveit_error_code"] == code
    assert result.diagnostics["moveit_error_name"] == name
    assert result.diagnostics["failure_category"] == category
    planner.close()
