"""P10 红→绿：IK 失败必须是 ``ik_no_solution``，不能变成 ``planner_exception``。

背景（P9 实测）：``MoveGroupPlanner._compute_ik()`` 里曾用

```python
record["failure_category"] = _ERROR_CODES.get(code, ("invalid", f"moveit_error_{code}"))[1]
```

而 ``moveit_error_codes.MOVEIT_ERROR_CODES`` 在 OMPL 基线一轮已重构成
``dict[int, MoveItErrorCode]``（值对象，不是 ``(status, category)`` 元组）。于是
**任何 IK 失败**都会抛 ``TypeError: 'MoveItErrorCode' object is not subscriptable``：

* 后端层：``plan()`` 只把 ``MoveGroupUnavailableError``/``ImportError``/``ValueError``/
  ``TimeoutError``/``RuntimeError`` 收敛成 typed 结果，``TypeError`` 直接穿透；
* 协调器层：``PlanningCoordinator._plan_shape()`` 的 ``except Exception`` 把它记成
  ``planner_exception``，站点可达性结论被整段吞掉（P9 第一次 probe 就是这样）。

本文件锁定两件事：

1. IK 失败（``-31 NO_IK_SOLUTION`` 或表外码）返回 ``status="unreachable"`` +
   ``failure_category="ik_no_solution"``，原始码保留在每个采样里；
2. 经公共协调器时失败类别仍是 ``ik_no_solution``，**不是** ``planner_exception``。

测试不依赖 ROS/MoveIt：只替换 ``_session.messages`` 与四个服务边界
（``_ensure_session``/``wait_ready``/``_load_scene``/``_call``），因此本机有没有
``rclpy`` 都跑同一套断言。
"""

from __future__ import annotations

import json
from types import SimpleNamespace

import pytest

from unilab_robot_contracts import (
    AccessMotionBlock,
    AccessMotionPlanCompiler,
    ActionKind,
    CartesianGoal,
    MotionProfileCatalog,
    PlanGoal,
    PlanRequest,
    RailAxis,
    ResolvedPointTarget,
    RobotModel,
    SafePoseConstraint,
)
from unilab_rail_mounted_arm.planning_coordinator import PlanningCoordinator
from unilab_robot_runtime import moveit_move_group
from unilab_robot_runtime.moveit_move_group import MoveGroupPlanner

URDF = """<?xml version="1.0"?><robot name="stub">
  <link name="world"/><link name="base_link"/><link name="link_1"/><link name="link_2"/>
  <joint name="rail" type="prismatic"><origin xyz="0 0 1" rpy="0 0 0"/>
    <parent link="world"/><child link="base_link"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="1" effort="10" velocity="0.05"/></joint>
  <joint name="joint_1" type="revolute"><origin xyz="0 0 0"/>
    <parent link="base_link"/><child link="link_1"/><axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="3.14"/></joint>
  <joint name="joint_2" type="revolute"><origin xyz="0 0 0"/>
    <parent link="link_1"/><child link="link_2"/><axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="3.14"/></joint>
</robot>"""

SRDF = """<?xml version="1.0"?><robot name="stub">
  <group name="arm"><chain base_link="base_link" tip_link="link_2"/></group>
  <group name="rail_axis"><joint name="rail"/></group>
  <disable_collisions link1="base_link" link2="link_1" reason="Adjacent"/>
</robot>"""

SAFE_Q = (0.0, 0.0)
STATION_RAIL_M = 0.42
RAIL_AXIS = RailAxis("rail", "world", (0.0, 1.0))
SITE_REF = "postproc.site.stub"


# --------------------------------------------------------------- 不依赖 ROS 的假消息


class _JointState:
    def __init__(self) -> None:
        self.name: list[str] = []
        self.position: list[float] = []


class _RobotState:
    def __init__(self) -> None:
        self.joint_state = _JointState()
        self.is_diff = False


class _Header:
    def __init__(self) -> None:
        self.frame_id = ""


class _PoseStamped:
    def __init__(self) -> None:
        self.header = _Header()
        self.pose = None


class _IkInner:
    def __init__(self) -> None:
        self.group_name = ""
        self.robot_state = None
        self.avoid_collisions = True
        self.ik_link_name = ""
        self.pose_stamped = None
        self.timeout = None


class _IkRequest:
    def __init__(self) -> None:
        self.ik_request = _IkInner()


def _messages() -> dict[str, object]:
    """只提供 ``_compute_ik`` 真正用到的三类消息。"""

    return {
        "RobotState": _RobotState,
        "PoseStamped": _PoseStamped,
        "GetPositionIK": SimpleNamespace(Request=_IkRequest),
    }


def planner_with_ik_code(monkeypatch, code: object) -> MoveGroupPlanner:
    """构造一个只回答 IK 的 ``MoveGroupPlanner``：每次 ``/compute_ik`` 都返回 ``code``。"""

    planner = MoveGroupPlanner()
    planner._session.messages = _messages()
    monkeypatch.setattr(planner, "_ensure_session", lambda: None)
    monkeypatch.setattr(planner, "wait_ready", lambda timeout_s=None: True)
    monkeypatch.setattr(planner, "_load_scene", lambda request: {"applied": True})
    monkeypatch.setattr(
        planner,
        "_call",
        lambda name, request, timeout_s=None: SimpleNamespace(
            error_code=SimpleNamespace(val=code)
        ),
    )
    # ``_pose_at``/``_duration`` 会 import geometry_msgs/builtin_interfaces；本测试只
    # 验证错误码映射，用等价占位替换即可（断言里不读这两个对象）。
    monkeypatch.setattr(moveit_move_group, "_pose_at", lambda pose: pose)
    monkeypatch.setattr(moveit_move_group, "_duration", lambda seconds: seconds)
    return planner


def robot_model() -> RobotModel:
    return RobotModel(
        model_id="stub-rail-2dof",
        urdf_xml=URDF,
        srdf_xml=SRDF,
        base_frame="world",
        flange_link="link_2",
        planning_group="arm",
        joint_names=("joint_1", "joint_2"),
        joint_limits=((-3.0, 3.0), (-3.0, 3.0)),
    )


def pose_goal() -> PlanGoal:
    return PlanGoal.from_pose(
        CartesianGoal(
            frame_ref="world",
            xyz_m=(0.4, 0.0, 0.6),
            orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
            link_name="link_2",
            position_tolerance_m=2e-3,
            orientation_tolerance_rad=2e-3,
        )
    )


def _profiles() -> MotionProfileCatalog:
    return MotionProfileCatalog.from_mapping(
        {
            "schema": "unilab.robot-motion-profiles/v1",
            "revision": "p10-ik-mapping-test@1.0.0",
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
                    "entry_profile_ref": "transit",
                    "approach_profile_ref": "linear",
                    "interaction_profile_ref": "linear",
                    "rail_profile_ref": "rail-safe",
                }
            },
        }
    )


def site_target() -> ResolvedPointTarget:
    return ResolvedPointTarget(
        target_ref=SITE_REF,
        device_ref="postproc",
        arm_target_ref=f"{SITE_REF}.grasp",
        rail_position_si=STATION_RAIL_M,
        access_block=AccessMotionBlock(
            block_ref="postproc.access.stub",
            interaction_target_ref=f"{SITE_REF}.grasp",
            approach_target_ref=f"{SITE_REF}.approach",
            entry_target_ref=f"{SITE_REF}.entry",
            transit_in=(),
            transit_out=(),
            return_mode="direct",
        ),
        source_chain=("test",),
        resolved_digest="0" * 64,
    )


def site_goals() -> dict[str, PlanGoal]:
    return {
        f"{SITE_REF}.entry": pose_goal(),
        f"{SITE_REF}.approach": pose_goal(),
        f"{SITE_REF}.grasp": pose_goal(),
    }


# ---------------------------------------------------------------- 后端层：typed 结论


@pytest.mark.parametrize(
    ("code", "sample_category"),
    [(-31, "no_ik_solution"), (-27, "goal_state_invalid"), (-1, "failure")],
)
def test_ik_failure_is_unreachable_not_an_exception(
    monkeypatch, code: int, sample_category: str
) -> None:
    """``MOVEIT_ERROR_CODES`` 里的码全部要能解释（旧下标写法在这里 TypeError）。

    ``-31`` 是 P9 观测到的真实失败码；``-27``/``-1`` 用来证明「表内任何已知码」
    都走同一条映射路径，不只是恰好一个分支。
    """

    planner = planner_with_ik_code(monkeypatch, code)
    request = PlanRequest(
        robot=robot_model(),
        start_q=SAFE_Q,
        goal=pose_goal(),
        timeout_s=1.0,
    )
    result = planner.plan(request)

    assert result.status == "unreachable"
    assert result.waypoints == ()
    assert result.diagnostics["failure_category"] == "ik_no_solution"
    ik = result.diagnostics["ik"]
    assert ik["checked"] is True
    assert ik["any_solution"] is False
    assert ik["seeds_probed"] >= 1
    # 每个采样都保留原始码与映射后的类别，不能因为映射失败丢掉原因。
    for sample in ik["samples"]:
        assert sample["error_code"] == code
        assert sample["failure_category"] == sample_category


def test_unrecognised_ik_error_code_is_categorised_not_raised(monkeypatch) -> None:
    """表外错误码同样不得抛异常：类别回落 ``moveit_error_<code>``。

    这一条在旧下标写法下**恰好也是绿的**（``dict.get`` 的默认值是元组，可下标），
    所以它守的是回落分支，不是 P9 的 TypeError；红证据来自上面三条已知码。
    """

    planner = planner_with_ik_code(monkeypatch, -4242)
    request = PlanRequest(
        robot=robot_model(),
        start_q=SAFE_Q,
        goal=pose_goal(),
        timeout_s=1.0,
    )
    result = planner.plan(request)

    assert result.status == "unreachable"
    for sample in result.diagnostics["ik"]["samples"]:
        assert sample["error_code"] == -4242
        assert sample["failure_category"] == "moveit_error_-4242"


# ------------------------------------------------------- 协调器层：不得变成 planner_exception


def test_coordinator_reports_ik_no_solution_not_planner_exception(monkeypatch) -> None:
    """同一个 IK 失败经公共协调器后仍是 ``ik_no_solution`` 且状态 ``unreachable``。"""

    planner = planner_with_ik_code(monkeypatch, -31)
    coordinator = PlanningCoordinator(
        planner=planner,
        robot=robot_model(),
        rail_axis=RAIL_AXIS,
        safe_pose=SafePoseConstraint(SAFE_Q),
        compiler=AccessMotionPlanCompiler(profiles=_profiles()),
    )
    result = coordinator.plan_workcell(
        site_target(),
        action=ActionKind.PICK,
        current_rail_m=STATION_RAIL_M,
        start_q=SAFE_Q,
        goal_for=site_goals(),
    )

    assert result.status == "unreachable"
    assert result.waypoints == ()
    assert result.duration_s == 0.0
    assert result.failure_phase == "entry"
    assert result.diagnostics["failure_category"] == "ik_no_solution"
    planned = [phase for phase in result.phases if phase.planned]
    assert [phase.phase_id for phase in planned] == ["entry"]
    assert planned[0].diagnostics["failure_category"] == "ik_no_solution"
    # 整份结果里不允许出现被吞掉的 planner_exception。
    assert "planner_exception" not in json.dumps(result.as_mapping())
