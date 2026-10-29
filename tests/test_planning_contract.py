"""Engine-agnostic planning contract and strict validation tests."""

from __future__ import annotations

import math

import pytest
from unilab_robot_contracts import (
    PLAN_STATUSES,
    AttachmentSpec,
    CartesianGoal,
    ClearanceConstraint,
    CollisionGeometry,
    CollisionObject,
    CollisionWorld,
    MotionPlanner,
    OrientationConstraintSpec,
    PlanConstraints,
    PlanGoal,
    PlanRequest,
    PlanResult,
    PlanStatus,
    PlanWaypoint,
    PlannerIdentity,
    RailAxis,
    RobotModel,
    SafePoseConstraint,
    SceneValidatingPlanner,
)

JOINTS = tuple(f"robot_cr5_joint_{index}" for index in range(1, 7))


def _urdf() -> str:
    joints = "".join(
        f"""
  <joint name="{name}" type="revolute">
    <parent link="{'base' if index == 1 else f'link{index - 1}'}"/>
    <child link="link{index}"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>"""
        for index, name in enumerate(JOINTS, start=1)
    )
    links = "".join(
        f'<link name="{f"base" if index == 0 else f"link{index}"}"/>'
        for index in range(7)
    )
    return f'<robot name="test_arm">{links}{joints}</robot>'


def _srdf() -> str:
    joints = "".join(f'<joint name="{name}"/>' for name in JOINTS)
    return f'<robot name="test_arm"><group name="robot_cr5_arm">{joints}</group></robot>'


def _model() -> RobotModel:
    return RobotModel(
        model_id="test-cr5",
        urdf_xml=_urdf(),
        srdf_xml=_srdf(),
        base_frame="base",
        flange_link="link6",
        planning_group="robot_cr5_arm",
        joint_names=JOINTS,
        joint_limits=((-3.14, 3.14),) * 6,
        kinematics={"robot_cr5_arm": {"kinematics_solver": "test/KDL"}},
    )


def _identity() -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _world() -> CollisionWorld:
    return CollisionWorld(
        objects=(
            CollisionObject(
                object_id="table",
                world_pose=_identity(),
                geometry=(CollisionGeometry("box", size_m=(1.0, 1.0, 0.1)),),
                role="static_world",
            ),
        )
    )


def _request(**overrides: object) -> PlanRequest:
    values: dict[str, object] = {
        "robot": _model(),
        "start_q": (0.0,) * 6,
        "goal": PlanGoal.from_joint((0.1,) * 6),
        "rail_axis": RailAxis("rail", "world", (0.0, 1.0)),
        "start_rail_m": 0.2,
        "collision_world": _world(),
        "constraints": PlanConstraints(
            orientation=(
                OrientationConstraintSpec(
                    "world",
                    "link6",
                    (0.0, 0.0, 0.0, 1.0),
                    0.05,
                    0.05,
                    math.pi,
                ),
            ),
            safe_pose=SafePoseConstraint((0.0,) * 6, 1e-3),
        ),
        "seeds": ((0.0,) * 6,),
        "timeout_s": 5.0,
    }
    values.update(overrides)
    return PlanRequest(**values)


def _identity_planner() -> PlannerIdentity:
    return PlannerIdentity(
        "MoveItPy",
        "moveit_py_fcl",
        "test",
        {"moveit": "test", "fcl": "test"},
    )


def _result(request: PlanRequest, **overrides: object) -> PlanResult:
    values: dict[str, object] = {
        "status": "success",
        "waypoints": (
            PlanWaypoint(request.start_q, 0.2, 0.0, "planner"),
            PlanWaypoint((0.1,) * 6, 0.2, 1.0, "planner"),
        ),
        "planner": _identity_planner(),
        "collision_checks": 2,
        "diagnostics": {"hardware": False},
    }
    values.update(overrides)
    return PlanResult(**values)


def test_plan_request_round_trips_and_digest_is_stable() -> None:
    """同一显式请求必须得到相同摘要，且不含本机路径。"""

    first = _request()
    second = _request()
    assert first.digest == second.digest
    assert first.planning_group == "robot_cr5_arm"
    assert first.goal.pose is None
    assert len(first.digest) == 64


def test_request_rejects_nonfinite_vectors_dimensions_and_limits() -> None:
    """有限数、维度和关节限位是硬校验。"""

    for field, value in (
        ("start_q", (0.0,) * 5),
        ("start_q", (float("nan"),) + (0.0,) * 5),
        ("start_q", (4.0,) + (0.0,) * 5),
        ("seeds", ((0.0,) * 5,)),
        ("timeout_s", 0.0),
        ("timeout_s", float("inf")),
    ):
        with pytest.raises((TypeError, ValueError)):
            _request(**{field: value})
    with pytest.raises(ValueError):
        _request(goal=PlanGoal.from_joint((0.0,) * 5))


def test_rail_and_arm_consistency_is_required() -> None:
    """导轨不能缺失、超限，或与六轴起点混为同一维度。"""

    with pytest.raises(ValueError):
        _request(start_rail_m=None)
    with pytest.raises(ValueError):
        _request(rail_axis=None)
    with pytest.raises(ValueError):
        _request(start_rail_m=1.1)
    with pytest.raises(ValueError):
        _request(start_rail_m=float("nan"))
    with pytest.raises(ValueError, match="goal_rail_m"):
        _request(rail_axis=None, start_rail_m=None, goal_rail_m=0.3)


def test_goal_rail_defaults_to_start_and_is_bounded() -> None:
    """``goal_rail_m`` 缺省等于起点导轨，显式值必须落在导轨限位内。"""

    request = _request()
    assert request.start_rail_m == 0.2
    assert request.goal_rail_m == 0.2
    assert request.goal_rail == 0.2
    assert request.rail_moves is False

    moving = _request(goal_rail_m=0.8)
    assert moving.goal_rail_m == 0.8
    assert moving.rail_moves is True

    with pytest.raises(ValueError, match="goal_rail_m"):
        _request(goal_rail_m=1.5)
    with pytest.raises(ValueError, match="goal_rail_m"):
        _request(goal_rail_m=float("nan"))
    with pytest.raises(ValueError, match="goal_rail_m"):
        _request(goal_rail_m=float("inf"))


def test_validate_against_checks_both_rail_endpoints() -> None:
    """执行前必须核对首/末路点的导轨值，不能只看起点。"""

    request = _request(goal_rail_m=0.8)
    with pytest.raises(ValueError, match="首点导轨"):
        _result(
            request,
            waypoints=(
                PlanWaypoint(request.start_q, 0.25, 0.0),
                PlanWaypoint((0.1,) * 6, 0.8, 1.0),
            ),
        ).validate_against(request)
    with pytest.raises(ValueError, match="末点导轨"):
        _result(
            request,
            waypoints=(
                PlanWaypoint(request.start_q, 0.2, 0.0),
                PlanWaypoint((0.1,) * 6, 0.5, 1.0),
            ),
        ).validate_against(request)
    moving = _result(
        request,
        waypoints=(
            PlanWaypoint(request.start_q, 0.2, 0.0),
            PlanWaypoint((0.1,) * 6, 0.8, 1.0),
        ),
    )
    moving.validate_against(request)


def test_success_result_rejects_empty_trajectory_and_bad_time_order() -> None:
    """成功但空轨迹、乱序时间、维度不一致都必须拒绝。"""

    request = _request()
    with pytest.raises(ValueError, match="空轨迹"):
        _result(request, waypoints=())
    with pytest.raises(ValueError, match="非递减"):
        _result(
            request,
            waypoints=(
                PlanWaypoint((0.0,) * 6, 0.2, 2.0),
                PlanWaypoint((0.1,) * 6, 0.2, 1.0),
            ),
        )
    with pytest.raises(ValueError, match="相同关节维度"):
        _result(
            request,
            waypoints=(
                PlanWaypoint((0.0,) * 6, 0.2, 0.0),
                PlanWaypoint((0.1,) * 5, 0.2, 1.0),
            ),
        )
    failed = _result(request, status="collision", waypoints=())
    assert failed.waypoints == ()


def test_result_digest_detects_tampering_and_validate_against_request() -> None:
    """摘要绑定内容；维度/导轨/端点错配必须在执行前失败。"""

    request = _request()
    result = _result(request)
    assert result.digest == _result(request).digest
    with pytest.raises(ValueError, match="digest"):
        PlanResult(
            status=result.status,
            waypoints=result.waypoints,
            planner=result.planner,
            collision_checks=result.collision_checks,
            safe_pose_used=result.safe_pose_used,
            diagnostics=result.diagnostics,
            digest="0" * 64,
        )
    result.validate_against(request)
    with pytest.raises(ValueError, match="rail_m"):
        _result(
            request,
            waypoints=(
                PlanWaypoint(request.start_q, 7.0, 0.0),
                PlanWaypoint((0.1,) * 6, 7.0, 1.0),
            ),
        ).validate_against(request)
    with pytest.raises(ValueError, match="首点"):
        _result(
            request,
            waypoints=(
                PlanWaypoint((0.2,) * 6, 0.2, 0.0),
                PlanWaypoint((0.1,) * 6, 0.2, 1.0),
            ),
        ).validate_against(request)


def test_world_attachments_and_clearance_are_typed_and_consistent() -> None:
    """附着对象必须在世界内，载荷必须全部被附着，几何必须精确。"""

    world = CollisionWorld(
        objects=(
            *_world().objects,
            CollisionObject(
                "tool",
                _identity(),
                (CollisionGeometry("cylinder", radius_m=0.01, length_m=0.1),),
                role="tool",
                attached_to_link="link6",
                touch_links=(),
            ),
        )
    )
    attachment = AttachmentSpec("tool", "link6", _identity(), ())
    request = _request(
        attachments=(attachment,),
        carried_body_ids=("tool",),
        held_material=None,
        collision_world=world,
    )
    assert request.attachments[0].body_id == "tool"
    with pytest.raises(ValueError, match="不存在"):
        _request(attachments=(AttachmentSpec("missing", "link6", _identity()),))
    with pytest.raises(ValueError, match="carried_body_ids"):
        _request(
            attachments=(attachment,),
            carried_body_ids=("other",),
            collision_world=world,
        )
    with pytest.raises(ValueError, match="索引越界"):
        CollisionGeometry(
            "mesh",
            vertices=((0.0, 0.0, 0.0),),
            triangles=((0, 1, 2),),
        )
    with pytest.raises(ValueError, match="正交"):
        CollisionObject(
            "bad",
            ((2.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            (CollisionGeometry("box", size_m=(1, 1, 1)),),
        )
    with pytest.raises(ValueError):
        ClearanceConstraint(-0.001)


def test_protocols_are_structural_and_pose_goal_uses_flange() -> None:
    """规划器端口不绑定类继承；未指定 link 的位姿目标默认法兰。"""

    class FakePlanner:
        def plan(self, request: PlanRequest) -> PlanResult:
            return _result(request)

    class FakeValidator:
        def validate(self, scene: CollisionWorld) -> dict[str, object]:
            return {"status": "valid"}

    assert isinstance(FakePlanner(), MotionPlanner)
    assert isinstance(FakeValidator(), SceneValidatingPlanner)
    request = _request(
        goal=PlanGoal.from_pose(
            CartesianGoal("world", (0.4, 0.0, 0.3), (0.0, 0.0, 0.0, 1.0))
        )
    )
    assert request.goal.pose is not None
    assert request.goal.pose.link_name == "link6"


def test_plan_status_is_a_closed_five_state_contract() -> None:
    """对外状态只能是五态；第六态和任意字符串必须在构造期失败。"""

    assert PLAN_STATUSES == {
        "success",
        "invalid",
        "collision",
        "unreachable",
        "timeout",
    }
    assert set(PlanStatus.__args__) == PLAN_STATUSES  # type: ignore[attr-defined]
    for status in sorted(PLAN_STATUSES):
        if status == "success":
            result = _result(_request(), status=status)
        else:
            result = _result(_request(), status=status, waypoints=())
        assert result.status == status
        assert len(result.digest) == 64
    with pytest.raises(ValueError, match="status"):
        _result(_request(), status="planned")


def test_waypoint_and_result_numbers_must_be_finite() -> None:
    """单个路点的 q/rail/time 和结果计数都不接受 NaN、Inf 或负时间。"""

    for overrides in (
        {"q": (float("nan"),) + (0.0,) * 5},
        {"q": (float("inf"),) + (0.0,) * 5},
        {"rail_m": float("nan")},
        {"rail_m": float("inf")},
        {"time_s": float("nan")},
        {"time_s": -0.1},
    ):
        with pytest.raises((TypeError, ValueError)):
            PlanWaypoint(**{"q": (0.0,) * 6, "rail_m": 0.0, "time_s": 0.0, **overrides})
    with pytest.raises(ValueError):
        _result(_request(), collision_checks=True)


def test_result_digest_binds_waypoints_and_diagnostics() -> None:
    """只改路点或诊断也必须改变摘要；伪造摘要必须被拒绝。"""

    request = _request()
    baseline = _result(request)
    moved = _result(
        request,
        waypoints=(
            PlanWaypoint(request.start_q, 0.2, 0.0),
            PlanWaypoint((0.2,) * 6, 0.2, 1.0),
        ),
    )
    assert moved.digest != baseline.digest
    rediagnosed = _result(request, diagnostics={"hardware": True})
    assert rediagnosed.digest != baseline.digest
    tampered = ("0" if baseline.digest[0] != "0" else "1") + baseline.digest[1:]
    with pytest.raises(ValueError, match="digest"):
        PlanResult(
            status=baseline.status,
            waypoints=baseline.waypoints,
            planner=baseline.planner,
            collision_checks=baseline.collision_checks,
            safe_pose_used=baseline.safe_pose_used,
            diagnostics=baseline.diagnostics,
            digest=tampered,
        )


def test_rail_axis_id_must_not_shadow_an_arm_joint() -> None:
    """导轨必须是独立轴：同名会把它并进六轴状态，必须显式拒绝。"""

    with pytest.raises(ValueError, match="导轨|rail"):
        _request(rail_axis=RailAxis(JOINTS[0], "world", (0.0, 1.0)))


def test_rail_is_a_separate_dimension_from_the_joint_vector() -> None:
    """路点 q 只能有机械臂 DOF 个分量，导轨走独立的 rail_m 字段。"""

    request = _request()
    with pytest.raises(ValueError, match="6"):
        PlanResult(
            status="success",
            waypoints=(PlanWaypoint((0.0,) * 7, 0.2, 0.0),),
            planner=_identity_planner(),
        ).validate_against(request)


def test_validate_against_rejects_rail_values_for_a_no_rail_request() -> None:
    """未声明导轨轴的请求只接受 rail_m=0，非零值说明结果错配。"""

    request = _request(rail_axis=None, start_rail_m=None)
    assert request.rail_axis is None
    zero = PlanResult(
        status="success",
        waypoints=(
            PlanWaypoint(request.start_q, 0.0, 0.0),
            PlanWaypoint((0.1,) * 6, 0.0, 1.0),
        ),
        planner=_identity_planner(),
    )
    zero.validate_against(request)
    nonzero = PlanResult(
        status="success",
        waypoints=(
            PlanWaypoint(request.start_q, 0.2, 0.0),
            PlanWaypoint((0.1,) * 6, 0.2, 1.0),
        ),
        planner=_identity_planner(),
    )
    with pytest.raises(ValueError, match="rail_m"):
        nonzero.validate_against(request)


def test_robot_model_rejects_bad_xml_and_duplicate_joints() -> None:
    """模型 XML、关节顺序和限位在构造期失败。"""

    with pytest.raises(ValueError, match="XML"):
        RobotModel(
            "bad",
            "<robot>",
            _srdf(),
            "base",
            "link6",
            "robot_cr5_arm",
            JOINTS,
        )
    with pytest.raises(ValueError, match="重复"):
        RobotModel(
            _model().model_id,
            _urdf(),
            _srdf(),
            "base",
            "link6",
            "robot_cr5_arm",
            (JOINTS[0],) * 6,
        )
    with pytest.raises(ValueError, match="下限"):
        RobotModel(
            _model().model_id,
            _urdf(),
            _srdf(),
            "base",
            "link6",
            "robot_cr5_arm",
            JOINTS,
            ((1.0, 0.0),) * 6,
        )


def test_robot_fixture_attachment_is_explicit_and_cannot_be_payload():
    fixture=AttachmentSpec("carriage","base",_identity(),purpose="robot_fixture")
    world=CollisionWorld(objects=(CollisionObject("carriage",_identity(),
        (CollisionGeometry("box",size_m=(.1,.1,.1)),),attached_to_link="base"),))
    request=_request(collision_world=world,attachments=(fixture,))
    assert request.as_mapping()["attachments"][0]["purpose"]=="robot_fixture"
    with pytest.raises(ValueError,match="fixture"):
        _request(collision_world=world,attachments=(fixture,),carried_body_ids=("carriage",))
    from dataclasses import replace
    with pytest.raises(ValueError,match="URDF"):
        _request(collision_world=world,attachments=(replace(fixture,link_name="absent"),))
    with pytest.raises(ValueError,match="法兰"):
        _request(collision_world=world,attachments=(replace(fixture,purpose="tool_or_payload"),))
