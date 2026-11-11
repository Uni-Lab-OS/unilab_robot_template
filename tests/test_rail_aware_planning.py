"""导轨感知规划的合同/后端测试（不需要 ROS/MoveIt 运行时）。

历史缺口：两个基线只把六个臂关节送进规划请求，导轨留在默认 0；本文件锁定
修复后的三条不变量：

* ``PlanRequest.goal_rail_m`` 缺省等于起点，绝不回落到 0；
* ``move_group`` 后端的机器人状态、导轨段拆相和整机碰撞采样都带导轨；
* 没有导轨规划组时显式拒绝，不把导轨塞进六轴组、也不静默按 0 规划。
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from unilab_robot_contracts.planning import (
    PlanGoal,
    PlanRequest,
    RailAxis,
    RobotModel,
)
from unilab_robot_runtime import moveit_planner
from unilab_robot_runtime.moveit_move_group import (
    MoveGroupPlanner,
    MoveGroupPlannerConfig,
    build_move_group_parameters,
    rail_planning_group,
)

URDF = """<?xml version="1.0"?><robot name="stub">
  <link name="world"/><link name="base_link"/><link name="link_1"/>
  <joint name="rail" type="prismatic"><origin xyz="0 0 1" rpy="0 0 0"/>
    <parent link="world"/><child link="base_link"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="1" effort="10" velocity="0.05"/></joint>
  <joint name="joint_1" type="revolute"><origin xyz="0 0 0"/>
    <parent link="base_link"/><child link="link_1"/><axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="3.14"/></joint>
</robot>"""

SRDF = """<?xml version="1.0"?><robot name="stub">
  <group name="arm"><chain base_link="base_link" tip_link="link_1"/></group>
  <group name="rail_axis"><joint name="rail"/></group>
  <disable_collisions link1="base_link" link2="link_1" reason="Adjacent"/>
</robot>"""

SRDF_WITHOUT_RAIL_GROUP = """<?xml version="1.0"?><robot name="stub">
  <group name="arm"><chain base_link="base_link" tip_link="link_1"/></group>
</robot>"""


def robot_model(srdf: str = SRDF) -> RobotModel:
    return RobotModel(
        model_id="stub-rail",
        urdf_xml=URDF,
        srdf_xml=srdf,
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
        "rail_axis": RailAxis("rail", "world", (0.0, 1.0)),
        "start_rail_m": 0.2,
        "timeout_s": 1.0,
    }
    payload.update(overrides)
    return PlanRequest(**payload)


def test_rail_planning_group_is_discovered_from_srdf() -> None:
    """只认关节集合恰好等于 {rail} 的组，混合组不算独立导轨组。"""

    assert rail_planning_group(robot_model(), "rail") == "rail_axis"
    assert rail_planning_group(
        robot_model(SRDF_WITHOUT_RAIL_GROUP), "rail"
    ) is None
    mixed = SRDF.replace(
        '<group name="rail_axis"><joint name="rail"/></group>',
        '<group name="rail_axis"><joint name="rail"/><joint name="joint_1"/></group>',
    )
    assert rail_planning_group(robot_model(mixed), "rail") is None
    assert rail_planning_group(robot_model(), "missing") is None


def test_parameters_add_rail_limits_group_and_totg_adapter() -> None:
    """move_group 参数必须带导轨限位、独立导轨组和 TOTG request adapter。"""

    config = MoveGroupPlannerConfig(planning_time_s=3.0)
    axis = RailAxis("rail", "world", (0.0, 1.0))
    params = build_move_group_parameters(robot_model(), config, rail_axis=axis)
    limits = params["robot_description_planning"]["joint_limits"]
    assert limits["rail"]["min_position"] == 0.0
    assert limits["rail"]["max_position"] == 1.0
    assert limits["rail"]["max_velocity"] == config.rail_velocity_m_s
    assert limits["rail"]["max_acceleration"] == config.rail_acceleration_m_s2
    assert "rail_axis" in params["ompl"]
    assert "simplify_solutions" not in params["ompl"]["rail_axis"]
    assert params["ompl"]["planner_configs"][config.planner_id]["simplify_solutions"] == 0
    assert params["ompl"]["request_adapters"] == (
        "default_planner_request_adapters/AddTimeOptimalParameterization"
    )
    # 没有导轨轴时不应凭空写出导轨配置。
    plain = build_move_group_parameters(robot_model(), config)
    assert "rail" not in plain["robot_description_planning"]["joint_limits"]
    assert "rail_axis" not in plain["ompl"]


class _JointState:
    def __init__(self) -> None:
        self.name: list[str] = []
        self.position: list[float] = []


class _RobotState:
    def __init__(self) -> None:
        self.joint_state = _JointState()
        self.is_diff = False


def test_state_carries_the_explicit_rail_coordinate() -> None:
    """构造的 RobotState 必须显式带导轨，不能只发六个臂关节。"""

    planner = MoveGroupPlanner()
    planner._session.messages = {"RobotState": _RobotState}
    req = request()
    state = planner._state(req, (0.5,), 0.3)
    assert state.joint_state.name == ["joint_1", "rail"]
    assert state.joint_state.position == [0.5, 0.3]
    start = planner._start_state(req)
    assert start.joint_state.position[-1] == 0.2
    # 不规划导轨不等于模型中没有导轨；禁止让真实轴落回默认零位。
    with pytest.raises(ValueError, match="missing passive joint state"):
        planner._state(request(rail_axis=None, start_rail_m=None), (0.5,))
    fixed_rail = planner._state(request(rail_axis=None, start_rail_m=None,
        options={"passive_joint_positions": {"rail": .2}}), (0.5,))
    assert dict(zip(fixed_rail.joint_state.name,fixed_rail.joint_state.position)) == {"joint_1": .5, "rail": .2}


def test_phases_split_a_rail_change_into_rail_then_arm() -> None:
    """导轨段先走独立导轨组，六轴段把导轨锁在段末值。"""

    planner = MoveGroupPlanner()
    moved = request(goal_rail_m=0.8)
    phases = planner._phases(moved)
    assert phases is not None and len(phases) == 2
    assert phases[0]["group"] == "rail_axis"
    assert phases[0]["joint_goal"] == {"rail": 0.8}
    assert phases[0]["rail_start"] == 0.2 and phases[0]["rail_goal"] == 0.8
    assert phases[1]["group"] == "arm"
    assert phases[1]["joint_goal"] == {"joint_1": 0.5}
    assert phases[1]["rail_start"] == phases[1]["rail_goal"] == 0.8

    fixed = planner._phases(request())
    assert fixed is not None and len(fixed) == 1
    assert fixed[0]["rail_start"] == fixed[0]["rail_goal"] == 0.2

    rail_only = planner._phases(
        request(goal=PlanGoal.from_joint([0.0]), goal_rail_m=0.9)
    )
    assert rail_only is not None and len(rail_only) == 1


def test_rail_change_without_a_rail_group_is_refused() -> None:
    """SRDF 没有独立导轨组时必须拒绝，而不是退回六轴组或导轨 0。"""

    planner = MoveGroupPlanner()
    req = request(robot=robot_model(SRDF_WITHOUT_RAIL_GROUP), goal_rail_m=0.8)
    assert planner._phases(req) is None


def test_moveitpy_backend_refuses_rail_motion_instead_of_defaulting_to_zero() -> None:
    """未实现导轨段的 MoveItPy 后端必须显式拒绝，不能把导轨当 0。"""

    result = moveit_planner.MoveItPlanner().plan(request(goal_rail_m=0.8))
    assert result.status == "invalid"
    assert result.diagnostics["failure_category"] == "rail_motion_not_supported"
    assert result.waypoints == ()


def test_rail_goal_tolerance_has_explicit_metre_unit():
    planner=MoveGroupPlanner()
    planner._session.messages={'Constraints':lambda:SimpleNamespace(joint_constraints=[]),'JointConstraint':SimpleNamespace}
    req=request(options={'joint_goal_tolerance_rad':.002,'rail_goal_tolerance_m':1e-6})
    result=planner._goal_constraints(req,joint_goal={'joint_1':.5,'rail':.65})
    by_name={v.joint_name:v for v in result.joint_constraints}
    assert by_name['joint_1'].position==.5
    assert by_name['joint_1'].tolerance_above==.002
    assert by_name['rail'].position==.65
    assert by_name['rail'].tolerance_above==1e-6
    with pytest.raises(ValueError,match='finite and positive'):
        planner._goal_constraints(request(options={'rail_goal_tolerance_m':float('nan')}),joint_goal={'rail':.65})
