"""P6/P7 导轨机械臂分相位规划协调器的红→绿测试（不需要 ROS/MoveIt）。

本文件锁定公共设计 22 的 rail-then-arm 规则：

* 相位顺序必须是 ``rail_transfer_safe`` → ``RAIL_MOVE`` → ``ARM_MOVE``；
* ``rail_transfer_safe`` 只规划机械臂，导轨锁在当前值；
* ``RAIL_MOVE`` 只规划导轨，机械臂锁在安全姿态；
* ``ARM_MOVE`` 锁导轨在站点值，且该值来自 ``ResolvedPointTarget``/站点，
  绝不回落到 0；
* 任一相位失败整个 WorkCell 计划失败，**不拼接部分轨迹**；
* 没有导轨时退化为纯机械臂相位；
* 失败状态/原始错误码透传，结果摘要绑定源计划与全部相位；
* 永远不把导轨并进关节向量做一次性 7 自由度联合规划。
"""

from __future__ import annotations

import dataclasses

import pytest

from unilab_robot_contracts import (
    AccessMotionBlock,
    AccessMotionPlanCompiler,
    ActionKind,
    MotionProfileCatalog,
    PlanGoal,
    PlanRequest,
    PlanResult,
    PlanWaypoint,
    PlannerIdentity,
    RailAxis,
    ResolvedPointTarget,
    ResolvedWorkCellPlan,
    RobotModel,
    SafePoseConstraint,
    WorkCellPhaseKind,
    WorkCellPlanPhase,
)
from unilab_rail_mounted_arm.planning_coordinator import (
    PlanningCoordinator,
    WorkCellPlanResult,
)

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
START_Q = (0.2, -0.1)
CURRENT_RAIL_M = 0.2
STATION_RAIL_M = 0.42
RAIL_AXIS = RailAxis("rail", "world", (0.0, 1.0))
IDENTITY = PlannerIdentity("fake.ompl", "recording_fake", "v1.0.0")

ARM_GOALS: dict[str, tuple[float, float]] = {
    "global.arm.standby": (0.1, 0.0),
    "rack.r1c1.entry": (0.2, 0.2),
    "rack.r1c1.approach": (0.3, 0.1),
    "rack.r1c1.interaction": (0.5, -0.4),
}


def robot_model() -> RobotModel:
    """返回两轴测试机械臂；导轨在合同里始终是独立轴。"""

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


def goals() -> dict[str, PlanGoal]:
    """把测试点位引用解析成关节目标。"""

    return {ref: PlanGoal.from_joint(values) for ref, values in ARM_GOALS.items()}


def _arm_phase(
    phase_id: str, target_ref: str, rail_m: float | None
) -> WorkCellPlanPhase:
    """构造一个机械臂相位，并携带编译器给的全块导轨值。"""

    return WorkCellPlanPhase(
        phase_id,
        WorkCellPhaseKind.ARM_MOVE,
        target_ref,
        "transit",
        rail_m,
        "empty",
    )


def _rail_phase(rail_m: float | None) -> WorkCellPlanPhase:
    """构造唯一的导轨换位相位。"""

    return WorkCellPlanPhase(
        "rail-move-and-settle",
        WorkCellPhaseKind.RAIL_MOVE,
        "rack.r1c1",
        "rail-safe",
        rail_m,
        "unchanged",
    )


def four_phase_plan(
    *, phase_rail: float | None = STATION_RAIL_M, digest: str = "a" * 64
) -> ResolvedWorkCellPlan:
    """构造编译器给出的人工计划：安全位 → 导轨 → 入口 → 交互。"""

    return ResolvedWorkCellPlan(
        "rack.r1c1",
        ActionKind.PICK,
        STATION_RAIL_M,
        (
            _arm_phase("rail-transfer-safe", "global.arm.rail_transfer_safe", phase_rail),
            _rail_phase(phase_rail),
            _arm_phase("entry", "rack.r1c1.entry", phase_rail),
            _arm_phase("interaction", "rack.r1c1.interaction", phase_rail),
        ),
        digest,
    )


def single_arm_plan(*, rail_m: float | None) -> ResolvedWorkCellPlan:
    """构造一个只有机械臂相位的计划。"""

    return ResolvedWorkCellPlan(
        "rack.r1c1",
        ActionKind.PICK,
        rail_m,
        (_arm_phase("entry", "rack.r1c1.entry", rail_m),),
        "d" * 64,
    )


def rail_only_plan() -> ResolvedWorkCellPlan:
    """构造只有导轨换位相位的计划（P7 rail-only 请求）。"""

    return ResolvedWorkCellPlan(
        "rack.r1c1",
        ActionKind.PICK,
        STATION_RAIL_M,
        (_rail_phase(STATION_RAIL_M),),
        "e" * 64,
    )


def _success(request: PlanRequest, identity: PlannerIdentity) -> PlanResult:
    """生成一个通过 ``validate_against`` 的两点成功轨迹。"""

    goal = (
        request.goal.joint_positions
        if request.goal.kind == "joint"
        else request.start_q
    )
    start_rail = 0.0 if request.start_rail_m is None else float(request.start_rail_m)
    goal_rail = 0.0 if request.goal_rail_m is None else float(request.goal_rail_m)
    return PlanResult(
        "success",
        (
            PlanWaypoint(tuple(request.start_q), start_rail, 0.0),
            PlanWaypoint(tuple(goal), goal_rail, 1.0),
        ),
        identity,
        collision_checks=1,
    )


def _arm_moves(request: PlanRequest) -> bool:
    """返回该请求是否要求六轴在段内移动。"""

    if request.goal.kind != "joint":
        return False
    assert request.goal.joint_positions is not None
    return max(
        abs(left - right)
        for left, right in zip(request.goal.joint_positions, request.start_q, strict=True)
    ) > 1e-9


class RecordingPlanner:
    """记录每个 ``PlanRequest`` 并返回确定成功轨迹的假后端。"""

    def __init__(
        self,
        *,
        fail_on: object | None = None,
        trajectory: object | None = None,
    ) -> None:
        """``fail_on`` 按请求返回失败结果；``trajectory`` 允许篡改轨迹。"""

        self.requests: list[PlanRequest] = []
        self.identity = IDENTITY
        self._fail_on = fail_on
        self._trajectory = trajectory

    def plan(self, request: PlanRequest) -> PlanResult:
        """记录请求；失败钩子优先，其次自定义轨迹，最后默认成功。"""

        self.requests.append(request)
        assert isinstance(request, PlanRequest)
        if self._fail_on is not None:
            failed = self._fail_on(request)  # type: ignore[operator]
            if failed is not None:
                return failed  # type: ignore[no-any-return]
        if self._trajectory is not None:
            return self._trajectory(request, self.identity)  # type: ignore[operator,no-any-return]
        return _success(request, self.identity)


def _coordinator(planner: object, *, rail: bool = True, **kwargs: object) -> PlanningCoordinator:
    """按装配形态构造协调器；无导轨时不注入安全姿态。"""

    return PlanningCoordinator(
        planner=planner,  # type: ignore[arg-type]
        robot=robot_model(),
        rail_axis=RAIL_AXIS if rail else None,
        safe_pose=SafePoseConstraint(SAFE_Q) if rail else None,
        **kwargs,  # type: ignore[arg-type]
    )


# --------------------------------------------------------------------- P6 顺序与锁


def test_phase_order_is_safe_then_rail_then_arm_with_exclusive_locks() -> None:
    """顺序固定为安全位 → 导轨 → 机械臂，且每个请求只动一个轴组。"""

    planner = RecordingPlanner()
    result = _coordinator(planner).plan(
        four_phase_plan(),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )

    assert result.status == "success"
    assert [phase.phase_id for phase in result.phases] == [
        "rail-transfer-safe",
        "rail-move-and-settle",
        "entry",
        "interaction",
    ]
    assert [phase.kind for phase in result.phases] == [
        "arm_move",
        "rail_move",
        "arm_move",
        "arm_move",
    ]
    assert len(planner.requests) == 4
    safe, rail, entry, interaction = planner.requests

    # 1. rail_transfer_safe：只动 arm，导轨锁在冻结当前位置（既不是 0 也不是站点）。
    assert safe.start_q == START_Q
    assert safe.goal.joint_positions == SAFE_Q
    assert safe.rail_moves is False
    assert safe.start_rail_m == safe.goal_rail_m == CURRENT_RAIL_M

    # 2. RAIL_MOVE：只动 rail，arm 保持在 safe_q。
    assert rail.rail_moves is True
    assert rail.start_rail_m == CURRENT_RAIL_M
    assert rail.goal_rail_m == STATION_RAIL_M
    assert rail.start_q == rail.goal.joint_positions == SAFE_Q

    # 3. ARM_MOVE：锁 rail，起点和终点都等于站点导轨值。
    for request in (entry, interaction):
        assert request.rail_moves is False
        assert request.start_rail_m == request.goal_rail_m == STATION_RAIL_M

    # 永不出现“arm + rail 同时移动”的联合请求，也不把导轨塞进关节向量。
    for request in planner.requests:
        assert len(request.start_q) == 2
        assert not (request.rail_moves and _arm_moves(request))

    assert result.waypoints[0].q == START_Q
    assert result.waypoints[0].time_s == 0.0
    assert result.duration_s == 4.0
    assert result.total_duration_s == 4.0
    assert result.planner == IDENTITY
    assert result.diagnostics["strategy"] == "rail_then_arm"
    assert result.source_plan_digest == "a" * 64


@pytest.mark.parametrize(
    ("status", "code"), [("collision", -16), ("unreachable", -31), ("timeout", -1)]
)
def test_planner_failure_status_and_raw_error_code_pass_through(
    status: str, code: int
) -> None:
    """失败状态和 MoveIt 原始码必须原样进入 WorkCell 结果。"""

    def fail_on_entry(request: PlanRequest) -> PlanResult | None:
        if (
            request.goal.kind == "joint"
            and request.goal.joint_positions == ARM_GOALS["rack.r1c1.entry"]
        ):
            return PlanResult(
                status,  # type: ignore[arg-type]
                (),
                IDENTITY,
                diagnostics={
                    "moveit_error_code": code,
                    "moveit_error_name": f"RAW_CODE_{code}",
                    "failure_category": "backend_failure",
                },
            )
        return None

    planner = RecordingPlanner(fail_on=fail_on_entry)
    result = _coordinator(planner).plan(
        four_phase_plan(),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )

    assert result.status == status and result.planner == IDENTITY
    assert result.failure_phase == "entry"
    failure = result.diagnostics["failure"]
    assert failure["phase_id"] == "entry"
    assert failure["status"] == status
    assert failure["planner_diagnostics"]["moveit_error_code"] == code
    assert failure["planner_diagnostics"]["moveit_error_name"] == f"RAW_CODE_{code}"


def test_any_phase_failure_fails_the_whole_plan_without_stitching() -> None:
    """任一段失败即整体失败；已完成的段只保留在相位明细里，不拼接成轨迹。"""

    def fail_on_entry(request: PlanRequest) -> PlanResult | None:
        if (
            request.goal.kind == "joint"
            and request.goal.joint_positions == ARM_GOALS["rack.r1c1.entry"]
        ):
            return PlanResult(
                "collision",
                (),
                IDENTITY,
                diagnostics={
                    "moveit_error_code": -16,
                    "moveit_error_name": "GOAL_IN_COLLISION",
                    "failure_category": "goal_in_collision",
                },
            )
        return None

    planner = RecordingPlanner(fail_on=fail_on_entry)
    result = _coordinator(planner).plan(
        four_phase_plan(),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )

    assert result.status == "collision" and result.planner == IDENTITY
    assert result.waypoints == ()
    assert result.duration_s == 0.0
    assert result.failure_phase == "entry"
    assert len(planner.requests) == 3  # 失败即停，不规划后续 interaction 相位
    assert [phase.status for phase in result.phases] == [
        "success",
        "success",
        "collision",
        None,
    ]
    assert result.phases[2].planned is True
    assert result.phases[3].planned is False
    # 已完成的相位轨迹只用于诊断，顶层可执行轨迹必须为空。
    assert result.phases[0].result is not None
    assert result.phases[0].result.status == "success"
    assert result.phases[0].waypoints
    assert result.phases[2].waypoints == ()

    failure = result.diagnostics["failure"]
    assert failure["phase_id"] == "entry"
    assert failure["category"] == "goal_in_collision"
    assert failure["planner_diagnostics"]["moveit_error_code"] == -16
    assert result.diagnostics["failure_category"] == "goal_in_collision"


def test_a_failure_result_cannot_carry_a_stitched_trajectory() -> None:
    """失败结果在类型层面就拒绝携带拼接轨迹，避免下游误执行。"""

    result = _coordinator(RecordingPlanner()).plan(
        four_phase_plan(),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )
    assert result.status == "success"
    with pytest.raises(ValueError):
        dataclasses.replace(result, status="collision")
    with pytest.raises(ValueError):
        WorkCellPlanResult(
            status="collision",
            plan_ref=result.plan_ref,
            action=result.action,
            phases=result.phases,
            waypoints=result.waypoints,
            planner=result.planner,
            source_plan_digest=result.source_plan_digest,
            failure_phase="entry",
            failure_reason="forged",
            diagnostics={},
        )


# --------------------------------------------------------------------- P7 rail 值


def test_rail_values_come_from_the_station_plan_and_never_default_to_zero() -> None:
    """导轨值只来自计划的站点值；缺失时失败关闭，且不向规划器发出 0 请求。"""

    planner = RecordingPlanner()
    result = _coordinator(planner).plan(
        four_phase_plan(),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )
    assert result.status == "success"
    assert planner.requests[1].goal_rail_m == STATION_RAIL_M
    assert all(request.start_rail_m is not None for request in planner.requests)
    assert all(
        request.goal_rail_m == STATION_RAIL_M for request in planner.requests[2:]
    )

    # 相位缺少站点导轨值时不得当成 0，也不得调用规划器。
    missing_phase_value = _coordinator(RecordingPlanner()).plan(
        four_phase_plan(phase_rail=None),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )
    assert missing_phase_value.status == "invalid"
    assert missing_phase_value.failure_phase == "rail-transfer-safe"
    assert (
        missing_phase_value.diagnostics["failure_category"]
        == "rail_position_missing"
    )
    assert missing_phase_value.waypoints == ()

    missing_effective = _coordinator(RecordingPlanner()).plan(
        dataclasses.replace(four_phase_plan(), effective_rail_position_si=None),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )
    assert missing_effective.status == "invalid"
    assert missing_effective.diagnostics["failure_category"] == "rail_position_missing"


def test_rail_only_plan_moves_the_rail_with_the_arm_locked_at_safe_q() -> None:
    """P7 rail-only 请求：只有导轨运动，arm 请求段内保持 safe_q。"""

    planner = RecordingPlanner()
    result = _coordinator(planner).plan(
        rail_only_plan(),
        start_q=SAFE_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for={},
    )

    assert result.status == "success"
    assert len(planner.requests) == 1
    request = planner.requests[0]
    assert request.rail_moves is True
    assert request.start_q == request.goal.joint_positions == SAFE_Q
    assert request.start_rail_m == CURRENT_RAIL_M
    assert request.goal_rail_m == STATION_RAIL_M
    assert [point.rail_m for point in result.waypoints] == [
        CURRENT_RAIL_M,
        STATION_RAIL_M,
    ]
    assert all(point.q == SAFE_Q for point in result.waypoints)
    assert result.duration_s == 1.0


def test_rail_move_requires_the_arm_to_already_be_at_the_safe_pose() -> None:
    """没有 rail_transfer_safe 时，只有起点已在安全位才允许移动导轨。"""

    planner = RecordingPlanner()
    result = _coordinator(planner).plan(
        rail_only_plan(),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for={},
    )
    assert result.status == "invalid"
    assert result.diagnostics["failure_category"] == "rail_move_requires_safe_pose"
    assert planner.requests == []
    assert result.waypoints == ()


def test_arm_move_before_the_rail_is_settled_is_refused() -> None:
    """机械臂相位要求导轨已由 RAIL_MOVE 到位，不能带着旧导轨值直接规划。"""

    planner = RecordingPlanner()
    result = _coordinator(planner).plan(
        single_arm_plan(rail_m=STATION_RAIL_M),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )
    assert result.status == "invalid"
    assert result.failure_phase == "entry"
    assert result.diagnostics["failure_category"] == "rail_not_settled"
    assert planner.requests == []
    assert result.waypoints == ()


def test_only_one_safe_transfer_phase_is_allowed_before_a_rail_move() -> None:
    """导轨换位前只允许一个安全位机械臂相位，禁止在旧导轨位置连续动臂。"""

    plan = ResolvedWorkCellPlan(
        "rack.r1c1",
        ActionKind.PICK,
        STATION_RAIL_M,
        (
            _arm_phase("pre-standby", "global.arm.standby", STATION_RAIL_M),
            _arm_phase("pre-entry", "rack.r1c1.entry", STATION_RAIL_M),
            _rail_phase(STATION_RAIL_M),
            _arm_phase("interaction", "rack.r1c1.interaction", STATION_RAIL_M),
        ),
        "9" * 64,
    )
    planner = RecordingPlanner()
    result = _coordinator(planner).plan(
        plan, start_q=START_Q, start_rail_m=CURRENT_RAIL_M, goal_for=goals()
    )
    assert result.status == "invalid"
    assert result.failure_phase == "pre-entry"
    assert result.diagnostics["failure_category"] == "phase_order_invalid"
    assert planner.requests == []


def test_a_station_rail_outside_the_axis_limits_is_refused_before_planning() -> None:
    """站点导轨值必须落在注入的 RailAxis 限位内，越界不调用规划器。"""

    planner = RecordingPlanner()
    result = _coordinator(planner).plan(
        dataclasses.replace(four_phase_plan(), effective_rail_position_si=2.0),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )
    assert result.status == "invalid"
    assert result.failure_phase == "rail-transfer-safe"
    assert result.diagnostics["failure_category"] == "rail_target_out_of_limits"
    assert planner.requests == []
    assert result.waypoints == ()


def test_an_out_of_limit_arm_goal_is_rejected_before_the_planner_runs() -> None:
    """目标超出关节限位时，请求在构造期被拒绝，后续相位不再规划。"""

    bad_goals = goals()
    bad_goals["rack.r1c1.entry"] = PlanGoal.from_joint([9.0, 0.0])
    planner = RecordingPlanner()
    result = _coordinator(planner).plan(
        four_phase_plan(),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=bad_goals,
    )
    assert result.status == "invalid"
    assert result.failure_phase == "entry"
    assert result.diagnostics["failure_category"] == "phase_request_invalid"
    assert len(planner.requests) == 2  # safe 与 rail 已规划，entry 请求被构造期拒绝
    assert result.waypoints == ()


def test_a_rail_plan_cannot_be_planned_without_a_rail_axis() -> None:
    """协调器未装配导轨时，带导轨的计划必须失败关闭而不是忽略导轨。"""

    planner = RecordingPlanner()
    result = _coordinator(planner, rail=False).plan(
        four_phase_plan(),
        start_q=START_Q,
        start_rail_m=None,
        goal_for=goals(),
    )
    assert result.status == "invalid"
    assert result.diagnostics["failure_category"] == "rail_axis_missing"
    assert planner.requests == []
    assert result.waypoints == ()


def test_a_plan_without_a_rail_degrades_to_arm_only_requests() -> None:
    """无导轨装配退化为纯 ARM_MOVE 请求，不制造导轨轴或 rail=0 约束。"""

    planner = RecordingPlanner()
    result = _coordinator(planner, rail=False).plan(
        single_arm_plan(rail_m=None),
        start_q=START_Q,
        start_rail_m=None,
        goal_for=goals(),
    )
    assert result.status == "success"
    assert [phase.kind for phase in result.phases] == ["arm_move"]
    assert len(planner.requests) == 1
    request = planner.requests[0]
    assert request.rail_axis is None
    assert request.start_rail_m is None
    assert request.goal_rail_m is None
    assert request.goal.joint_positions == ARM_GOALS["rack.r1c1.entry"]
    assert all(point.rail_m == 0.0 for point in result.waypoints)
    assert result.diagnostics["strategy"] == "arm_only"
    assert result.duration_s == 1.0


# ------------------------------------------------------------------ 结果校验与失败关闭


def test_result_that_moves_the_arm_during_a_rail_phase_is_rejected() -> None:
    """RAIL_MOVE 期间任何中间路点移动机械臂都视为锁失效。"""

    def drift_arm(request: PlanRequest, identity: PlannerIdentity) -> PlanResult:
        if request.rail_moves:
            return PlanResult(
                "success",
                (
                    PlanWaypoint(tuple(request.start_q), float(request.start_rail_m), 0.0),
                    PlanWaypoint((0.4, 0.4), float(request.start_rail_m), 0.5),
                    PlanWaypoint(
                        tuple(request.goal.joint_positions),  # type: ignore[arg-type]
                        float(request.goal_rail_m),
                        1.0,
                    ),
                ),
                identity,
            )
        return _success(request, identity)

    planner = RecordingPlanner(trajectory=drift_arm)
    result = _coordinator(planner).plan(
        four_phase_plan(),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )
    assert result.status == "invalid"
    assert result.failure_phase == "rail-move-and-settle"
    assert result.diagnostics["failure_category"] == "rail_lock_violation"
    assert result.waypoints == ()
    assert result.phases[1].waypoints == ()


def test_result_that_moves_the_rail_during_an_arm_phase_is_rejected() -> None:
    """ARM_MOVE 期间任何中间路点移动导轨都视为锁失效。"""

    def drift_rail(request: PlanRequest, identity: PlannerIdentity) -> PlanResult:
        if (
            request.goal.kind == "joint"
            and request.goal.joint_positions == ARM_GOALS["rack.r1c1.entry"]
        ):
            station = float(request.start_rail_m)
            return PlanResult(
                "success",
                (
                    PlanWaypoint(tuple(request.start_q), station, 0.0),
                    PlanWaypoint(tuple(request.start_q), station + 0.05, 0.5),
                    PlanWaypoint(
                        tuple(request.goal.joint_positions),  # type: ignore[arg-type]
                        station,
                        1.0,
                    ),
                ),
                identity,
            )
        return _success(request, identity)

    planner = RecordingPlanner(trajectory=drift_rail)
    result = _coordinator(planner).plan(
        four_phase_plan(),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )
    assert result.status == "invalid"
    assert result.failure_phase == "entry"
    assert result.diagnostics["failure_category"] == "rail_lock_violation"
    assert result.waypoints == ()


def test_a_result_that_disagrees_with_its_request_fails_closed() -> None:
    """``validate_against`` 失败必须整体失败，不能把错配轨迹交给执行器。"""

    def inconsistent(
        request: PlanRequest, identity: PlannerIdentity
    ) -> PlanResult:
        return PlanResult(
            "success",
            (
                PlanWaypoint(
                    (request.start_q[0] + 0.25, request.start_q[1]),
                    float(request.start_rail_m or 0.0),
                    0.0,
                ),
                PlanWaypoint(
                    tuple(request.goal.joint_positions),  # type: ignore[arg-type]
                    float(request.goal_rail_m or 0.0),
                    1.0,
                ),
            ),
            identity,
        )

    planner = RecordingPlanner(trajectory=inconsistent)
    result = _coordinator(planner).plan(
        four_phase_plan(),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )
    assert result.status == "invalid" and result.planner == IDENTITY
    assert result.failure_phase == "rail-transfer-safe"
    assert result.diagnostics["failure_category"] == "phase_result_inconsistent"
    assert result.waypoints == ()
    assert len(planner.requests) == 1


def test_an_injected_waypoint_validator_turns_collisions_into_whole_plan_failure() -> None:
    """可选逐路点碰撞校验失败时，状态收敛为 collision 且不拼接轨迹。"""

    def reject_entry(
        request: PlanRequest, waypoint: PlanWaypoint
    ) -> tuple[bool, dict[str, object]]:
        if (
            request.goal.kind == "joint"
            and request.goal.joint_positions == ARM_GOALS["rack.r1c1.entry"]
        ):
            return False, {"object_id": "deck.rack", "clearance_m": -0.01}
        return True, {}

    planner = RecordingPlanner()
    result = _coordinator(planner, waypoint_validator=reject_entry).plan(
        four_phase_plan(),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )
    assert result.status == "collision"
    assert result.failure_phase == "entry"
    assert result.diagnostics["failure_category"] == "waypoint_collision"
    assert result.waypoints == ()
    failure = result.diagnostics["failure"]
    assert failure["planner_diagnostics"]["waypoint_index"] == 0
    assert failure["planner_diagnostics"]["validator"]["object_id"] == "deck.rack"


def test_planner_identity_is_recorded_and_must_stay_stable() -> None:
    """结果必须带规划器身份；同一 WorkCell 计划内身份变化视为失败关闭。"""

    planner = RecordingPlanner()
    result = _coordinator(planner).plan(
        four_phase_plan(),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )
    assert result.planner == IDENTITY
    assert result.phases[0].result is not None
    assert result.phases[0].result.planner == IDENTITY

    class ShiftingIdentityPlanner:
        def __init__(self) -> None:
            self.requests: list[PlanRequest] = []

        def plan(self, request: PlanRequest) -> PlanResult:
            self.requests.append(request)
            identity = PlannerIdentity(
                "fake.ompl", "variant_backend", str(len(self.requests))
            )
            return _success(request, identity)

    shifting = ShiftingIdentityPlanner()
    failed = _coordinator(shifting).plan(
        four_phase_plan(),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )
    assert failed.status == "invalid"
    assert failed.failure_phase == "rail-move-and-settle"
    assert failed.diagnostics["failure_category"] == "planner_identity_mismatch"
    assert failed.waypoints == ()
    assert failed.planner == PlannerIdentity("fake.ompl", "variant_backend", "1")


def test_a_broken_planner_fails_closed_without_a_stitched_trajectory() -> None:
    """后端返回非结果或抛异常时，协调器失败关闭并保留原因。"""

    class NonePlanner:
        def plan(self, request: PlanRequest) -> None:
            return None

    class RaisingPlanner:
        def plan(self, request: PlanRequest) -> PlanResult:
            raise RuntimeError("move_group unavailable")

    none_result = _coordinator(NonePlanner()).plan(
        four_phase_plan(),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )
    assert none_result.status == "invalid"
    assert none_result.failure_phase == "rail-transfer-safe"
    assert none_result.diagnostics["failure_category"] == "planner_result_invalid"
    assert none_result.waypoints == ()

    raising = _coordinator(RaisingPlanner()).plan(
        four_phase_plan(),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )
    assert raising.status == "invalid"
    assert raising.diagnostics["failure_category"] == "planner_exception"
    assert "move_group unavailable" in raising.diagnostics["failure"][
        "planner_diagnostics"
    ]["error"]
    assert raising.waypoints == ()


# --------------------------------------------------------------------- digest 与装配


def test_result_digest_binds_the_source_plan_and_rejects_tampering() -> None:
    """结果摘要绑定源计划与相位内容；伪造字段在构造期被拒绝。"""

    result_a = _coordinator(RecordingPlanner()).plan(
        four_phase_plan(digest="a" * 64),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )
    result_b = _coordinator(RecordingPlanner()).plan(
        four_phase_plan(digest="b" * 64),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )
    assert result_a.status == result_b.status == "success"
    assert result_a.source_plan_digest == "a" * 64
    assert result_b.source_plan_digest == "b" * 64
    assert len(result_a.digest) == 64
    assert result_a.digest != result_b.digest
    repeated = _coordinator(RecordingPlanner()).plan(
        four_phase_plan(digest="a" * 64),
        start_q=START_Q,
        start_rail_m=CURRENT_RAIL_M,
        goal_for=goals(),
    )
    assert repeated.digest == result_a.digest  # 同一输入确定性复现

    with pytest.raises(ValueError):
        dataclasses.replace(result_a, plan_ref="forged")
    with pytest.raises(ValueError):
        dataclasses.replace(result_a, source_plan_digest="c" * 64)


def test_a_rail_coordinator_requires_an_explicit_in_range_start_rail() -> None:
    """导轨装配不得把缺省 start_rail_m 解释成 0，越界同样拒绝。"""

    coordinator = _coordinator(RecordingPlanner())
    with pytest.raises(ValueError):
        coordinator.plan(
            four_phase_plan(),
            start_q=START_Q,
            start_rail_m=None,
            goal_for=goals(),
        )
    with pytest.raises(ValueError):
        coordinator.plan(
            four_phase_plan(),
            start_q=START_Q,
            start_rail_m=2.0,
            goal_for=goals(),
        )


def test_a_rail_assembly_requires_a_safe_pose() -> None:
    """导轨换位的安全姿态是构造期硬约束，不能用空值绕过。"""

    with pytest.raises(ValueError):
        PlanningCoordinator(
            planner=RecordingPlanner(),
            robot=robot_model(),
            rail_axis=RAIL_AXIS,
            safe_pose=None,
        )


# ------------------------------------------------------------------- 编译器端到端


def _profiles() -> MotionProfileCatalog:
    """返回与 AccessMotionPlan 测试相同的运动策略目录。"""

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


def _resolved_target() -> ResolvedPointTarget:
    """构造一个带站点导轨值的已解析 D9-1S 目标。"""

    return ResolvedPointTarget(
        "rack.r1c1",
        "deck.rack",
        "rack.r1c1.interaction",
        STATION_RAIL_M,
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
        "f" * 64,
    )


def test_compiled_access_plan_runs_safe_then_rail_then_arm_from_the_station_value() -> None:
    """真实 ``AccessMotionPlanCompiler`` 输出与协调器相位/导轨锁一致。"""

    compiler = AccessMotionPlanCompiler(profiles=_profiles())
    target = _resolved_target()
    compiled = compiler.compile(
        target,
        action=ActionKind.PICK,
        policy_ref="standard",
        current_rail_position_si=CURRENT_RAIL_M,
        has_rail=True,
    )
    assert compiled.effective_rail_position_si == STATION_RAIL_M

    planner = RecordingPlanner()
    coordinator = PlanningCoordinator(
        planner=planner,
        robot=robot_model(),
        rail_axis=RAIL_AXIS,
        safe_pose=SafePoseConstraint(SAFE_Q),
        compiler=compiler,
    )
    result = coordinator.plan_workcell(
        target,
        action=ActionKind.PICK,
        current_rail_m=CURRENT_RAIL_M,
        start_q=START_Q,
        goal_for=goals(),
    )

    assert result.status == "success"
    assert result.source_plan_digest == compiled.digest
    motion = [phase for phase in result.phases if phase.planned]
    assert [phase.phase_id for phase in motion[:3]] == [
        "rail-transfer-safe",
        "rail-move-and-settle",
        "transit-in-1",
    ]
    assert [phase.kind for phase in motion[:3]] == [
        "arm_move",
        "rail_move",
        "arm_move",
    ]
    assert len(motion) == 9  # 8 个 ARM_MOVE + 1 个 RAIL_MOVE
    assert len(planner.requests) == 9

    # 站点导轨值来自 ResolvedPointTarget，不是 0，也不把导轨并进关节向量。
    assert planner.requests[0].start_rail_m == planner.requests[0].goal_rail_m == 0.2
    assert planner.requests[1].goal_rail_m == STATION_RAIL_M
    assert all(
        request.goal_rail_m == STATION_RAIL_M for request in planner.requests[2:]
    )
    for request in planner.requests:
        assert len(request.start_q) == 2
        assert not (request.rail_moves and _arm_moves(request))

    assert result.duration_s == 9.0
    assert result.planner == IDENTITY
