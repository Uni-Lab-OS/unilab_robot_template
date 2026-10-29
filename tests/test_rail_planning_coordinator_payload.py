"""P18：公共协调器把夹持载荷注入每个相位请求的红测试（不需要 ROS/MoveIt）。

锁定的不变量：

* 装配了载荷时，协调器为**每个**运动相位构造的 ``PlanRequest`` 都携带同一
  个 ``PayloadAttachment``（attachments / carried_body_ids / held_material /
  payload 四处一致），并输出 attachment digest；
* 规划器必须回显 payload identity/profile/attachment digest，否则整个
  WorkCell 计划 ``invalid``（``payload_not_recorded``），绝不拼接；
* 回显与请求不一致时 ``invalid``（``payload_identity_mismatch``）；
* 相邻段端点关节不连续（2π 重标定）时 ``invalid``（``joint_discontinuity``），
  即使每段本身都是 ``success``；
* 没有装配载荷时行为与 P6/P7 完全一致；
* 载荷不在协调器自己的碰撞世界里时在构造期失败（fail fast）。
"""

from __future__ import annotations

import math

import pytest

from unilab_robot_contracts import (
    ActionKind,
    AttachmentSpec,
    CollisionGeometry,
    CollisionObject,
    CollisionPrimitive,
    CollisionWorld,
    JointContinuityLimits,
    PayloadAttachment,
    PayloadCollisionProfile,
    PlanGoal,
    PlanRequest,
    PlanResult,
    PlanWaypoint,
    PlannerIdentity,
    ResolvedWorkCellPlan,
    RigidTransform,
    RobotModel,
    WorkCellPhaseKind,
    WorkCellPlanPhase,
)
from unilab_rail_mounted_arm.planning_coordinator import (
    PlanningCoordinator,
    WorkCellPlanResult,
)

URDF = """<?xml version="1.0"?><robot name="stub">
  <link name="base_link"/><link name="link_1"/><link name="link_2"/>
  <joint name="joint_1" type="revolute"><origin xyz="0 0 0"/>
    <parent link="base_link"/><child link="link_1"/><axis xyz="0 0 1"/>
    <limit lower="-6.5" upper="6.5" effort="10" velocity="3.14"/></joint>
  <joint name="joint_2" type="revolute"><origin xyz="0 0 0"/>
    <parent link="link_1"/><child link="link_2"/><axis xyz="0 0 1"/>
    <limit lower="-6.5" upper="6.5" effort="10" velocity="3.14"/></joint>
</robot>"""

SRDF = """<?xml version="1.0"?><robot name="stub">
  <group name="arm"><chain base_link="base_link" tip_link="link_2"/></group>
</robot>"""

FLANGE = "link_2"
PAYLOAD_BODY = "payload/beaker-500ml"
PROFILE_REF = "szlab_beaker_500ml@v1"
PROFILE_DIGEST = "c" * 64
FLANGE_FROM_BODY = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, -0.031944),
    (0.0, 0.0, 0.0, 1.0),
)
IDENTITY = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
)
START_Q = (0.2, -0.1)
IDENTITY_PLANNER = PlannerIdentity("fake.ompl", "recording_fake", "v1.0.0")
ARM_GOALS: dict[str, tuple[float, float]] = {
    "rack.r1c1.entry": (0.2, 0.2),
    "rack.r1c1.interaction": (0.5, -0.4),
}


def robot_model() -> RobotModel:
    """返回两轴测试机械臂。

    关节限位取 ±6.5 rad（CR7 实测 ±6.27），这样 2π 重标定在**位置限位内**
    仍然合法，连续性检查才有机会独立咬住它，而不是被关节限位先挡住。
    """

    return RobotModel(
        model_id="stub-2dof",
        urdf_xml=URDF,
        srdf_xml=SRDF,
        base_frame="world",
        flange_link=FLANGE,
        planning_group="arm",
        joint_names=("joint_1", "joint_2"),
        joint_limits=((-6.5, 6.5), (-6.5, 6.5)),
    )


def profile() -> PayloadCollisionProfile:
    """返回烧杯的保守圆柱 Profile。"""

    return PayloadCollisionProfile(
        profile_ref=PROFILE_REF,
        digest=PROFILE_DIGEST,
        mass_kg=0.2,
        center_of_mass_m=(0.0, 0.0, 0.0),
        root_to_grasp=RigidTransform((0.0, 0.0, 0.031944), (0.0, 0.0, 0.0, 1.0)),
        collision_primitives=(
            CollisionPrimitive(
                "cylinder", (0.118, 0.0483), RigidTransform.identity()
            ),
        ),
    )


def attachment(**overrides: object) -> AttachmentSpec:
    """返回载荷在法兰上的固定安装关系。"""

    values: dict[str, object] = {
        "body_id": PAYLOAD_BODY,
        "link_name": FLANGE,
        "flange_from_body": FLANGE_FROM_BODY,
        "touch_links": ("link_1",),
    }
    values.update(overrides)
    return AttachmentSpec(**values)  # type: ignore[arg-type]


def payload(**overrides: object) -> PayloadAttachment:
    """返回绑定到附件与 Profile 的载荷。"""

    values: dict[str, object] = {"attachment": attachment(), "profile": profile()}
    values.update(overrides)
    return PayloadAttachment(**values)  # type: ignore[arg-type]


def world(*, include_payload: bool = True) -> CollisionWorld:
    """返回带可选载荷对象的碰撞世界。"""

    objects: list[CollisionObject] = [
        CollisionObject(
            "table",
            IDENTITY,
            (CollisionGeometry("box", size_m=(1.0, 1.0, 0.1)),),
            role="static_world",
        )
    ]
    if include_payload:
        objects.append(
            CollisionObject(
                PAYLOAD_BODY,
                FLANGE_FROM_BODY,
                (CollisionGeometry("cylinder", radius_m=0.0483, length_m=0.118),),
                role="payload",
                attached_to_link=FLANGE,
                touch_links=("link_1",),
            )
        )
    return CollisionWorld(objects=tuple(objects))


def two_phase_plan() -> ResolvedWorkCellPlan:
    """两个 ARM_MOVE 相位（无导轨），用于观察逐段载荷注入与连续性。"""

    return ResolvedWorkCellPlan(
        "rack.r1c1",
        ActionKind.PICK,
        None,
        (
            WorkCellPlanPhase(
                "entry",
                WorkCellPhaseKind.ARM_MOVE,
                "rack.r1c1.entry",
                "transit",
                None,
                "empty",
            ),
            WorkCellPlanPhase(
                "interaction",
                WorkCellPhaseKind.ARM_MOVE,
                "rack.r1c1.interaction",
                "transit",
                None,
                "empty",
            ),
        ),
        "a" * 64,
    )


def goals() -> dict[str, PlanGoal]:
    """把测试点位引用解析成关节目标。"""

    return {ref: PlanGoal.from_joint(values) for ref, values in ARM_GOALS.items()}


def _success(request: PlanRequest, identity: PlannerIdentity) -> PlanResult:
    """生成一个通过 ``validate_against`` 的两点成功轨迹（不回显载荷）。"""

    goal = (
        request.goal.joint_positions
        if request.goal.kind == "joint"
        else request.start_q
    )
    return PlanResult(
        "success",
        (
            PlanWaypoint(tuple(request.start_q), 0.0, 0.0),
            PlanWaypoint(tuple(goal), 0.0, 1.0),
        ),
        identity,
        collision_checks=1,
    )


class RecordingPlanner:
    """记录每个请求，并按 ``payload_echo`` 决定是否回显载荷身份。"""

    def __init__(
        self,
        *,
        payload_echo: bool = True,
        echo_ref: str | None = None,
        trajectory: object | None = None,
        fail_on: object | None = None,
    ) -> None:
        self.requests: list[PlanRequest] = []
        self.identity = IDENTITY_PLANNER
        self._payload_echo = payload_echo
        self._echo_ref = echo_ref
        self._trajectory = trajectory
        self._fail_on = fail_on

    def plan(self, request: PlanRequest) -> PlanResult:
        self.requests.append(request)
        if self._fail_on is not None:
            failed = self._fail_on(request)  # type: ignore[operator]
            if failed is not None:
                return failed  # type: ignore[no-any-return]
        if self._trajectory is not None:
            return self._trajectory(request, self.identity)  # type: ignore[operator,no-any-return]
        result = _success(request, self.identity)
        if request.payload is None or not self._payload_echo:
            return result
        if self._echo_ref is None:
            return result.with_payload(request)
        # 故意回显一个不同的 identity：协调器必须把它判成 invalid。
        return PlanResult(
            status=result.status,
            waypoints=result.waypoints,
            planner=result.planner,
            collision_checks=result.collision_checks,
            diagnostics=result.diagnostics,
            payload_ref=self._echo_ref,
            payload_profile_digest=request.payload.profile_digest,
            attachment_digest=request.payload.digest,
        )


def _coordinator(planner: object, **kwargs: object) -> PlanningCoordinator:
    if "collision_world" not in kwargs:
        # 没有装配载荷时，世界里也不能留一个"已附着"的对象（那本身就是非法场景）。
        kwargs["collision_world"] = world(
            include_payload=kwargs.get("payload") is not None
        )
    return PlanningCoordinator(
        planner=planner,  # type: ignore[arg-type]
        robot=robot_model(),
        **kwargs,  # type: ignore[arg-type]
    )


def _run(coordinator: PlanningCoordinator) -> WorkCellPlanResult:
    return coordinator.plan(
        two_phase_plan(), start_q=START_Q, start_rail_m=None, goal_for=goals()
    )


def test_coordinator_injects_the_same_payload_into_every_phase() -> None:
    """每个相位请求都带同一载荷，结果输出 payload/attachment digest。"""

    payload_attachment = payload()
    planner = RecordingPlanner()
    coordinator = _coordinator(planner, payload=payload_attachment)
    result = _run(coordinator)

    assert result.status == "success"
    assert len(planner.requests) == 2
    for request in planner.requests:
        assert request.payload is payload_attachment
        assert request.attachments == (attachment(),)
        assert request.carried_body_ids == (PAYLOAD_BODY,)
        assert request.held_material == PROFILE_REF
    assert result.payload_ref == PROFILE_REF
    assert result.attachment_digest == payload_attachment.digest
    assert result.as_mapping()["attachment_digest"] == payload_attachment.digest
    for phase in result.phases:
        assert phase.as_mapping()["payload_ref"] == PROFILE_REF
        assert phase.as_mapping()["attachment_digest"] == payload_attachment.digest


def test_coordinator_without_payload_behaves_as_before() -> None:
    """没有装配载荷时，请求与结果都不出现任何 payload 痕迹。"""

    planner = RecordingPlanner()
    result = _run(_coordinator(planner))

    assert result.status == "success"
    assert result.payload_ref is None
    assert result.attachment_digest is None
    assert "payload" not in result.as_mapping()
    for request in planner.requests:
        assert request.payload is None
        assert request.attachments == ()
        assert request.carried_body_ids == ()
        assert request.held_material is None


def test_planner_that_ignores_payload_makes_the_plan_invalid() -> None:
    """规划器不回显载荷身份 = 没有证明载荷进过它的场景，必须失败关闭。"""

    result = _run(
        _coordinator(RecordingPlanner(payload_echo=False), payload=payload())
    )

    assert result.status == "invalid"
    assert result.waypoints == ()
    assert result.failure_phase == "entry"
    assert result.diagnostics["failure_category"] == "payload_not_recorded"
    assert result.attachment_digest == payload().digest


def test_planner_echoing_a_different_payload_identity_is_rejected() -> None:
    """回显的 payload identity 与请求不一致时整体 invalid，不拼接轨迹。"""

    result = _run(
        _coordinator(
            RecordingPlanner(echo_ref="other-payload@v1"), payload=payload()
        )
    )

    assert result.status == "invalid"
    assert result.waypoints == ()
    assert result.diagnostics["failure_category"] == "payload_identity_mismatch"


def test_two_pi_relabel_inside_a_phase_is_joint_discontinuity() -> None:
    """每段都 success 但段内出现 2π 整圈重标定，整条链必须 invalid。"""

    def relabel(request: PlanRequest, identity: PlannerIdentity) -> PlanResult:
        goal = request.goal.joint_positions
        assert goal is not None
        bumped = (request.start_q[0] + 2.0 * math.pi, request.start_q[1])
        return PlanResult(
            "success",
            (
                PlanWaypoint(tuple(request.start_q), 0.0, 0.0),
                PlanWaypoint(bumped, 0.0, 0.5),
                PlanWaypoint(tuple(goal), 0.0, 1.0),
            ),
            identity,
        ).with_payload(request)

    result = _run(
        _coordinator(
            RecordingPlanner(trajectory=relabel), payload=payload()
        )
    )

    assert result.status == "invalid"
    assert result.waypoints == ()
    assert result.diagnostics["failure_category"] == "joint_discontinuity"
    report = result.diagnostics["joint_continuity"]
    assert report["ok"] is False
    assert report["violations"][0]["kind"] == "two_pi_jump"
    assert report["violations"][0]["scope"] == "waypoint"
    assert report["violations"][0]["joint_index"] == 0


def test_planner_failure_keeps_its_status_and_payload_digest() -> None:
    """规划器自身失败优先于载荷回显；结果仍记录本次请求的 attachment digest。"""

    def fail_on_entry(request: PlanRequest) -> PlanResult | None:
        if (
            request.goal.kind == "joint"
            and request.goal.joint_positions == ARM_GOALS["rack.r1c1.entry"]
        ):
            return PlanResult(
                "collision",
                (),
                IDENTITY_PLANNER,
                diagnostics={"failure_category": "goal_in_collision"},
            )
        return None

    result = _run(
        _coordinator(RecordingPlanner(fail_on=fail_on_entry), payload=payload())
    )

    assert result.status == "collision"
    assert result.failure_phase == "entry"
    assert result.waypoints == ()
    assert result.attachment_digest == payload().digest


def test_coordinator_rejects_payload_missing_from_its_collision_world() -> None:
    """载荷不在协调器碰撞世界里时构造期就拒绝，不把错误拖到规划中途。"""

    with pytest.raises(ValueError, match="碰撞世界"):
        PlanningCoordinator(
            planner=RecordingPlanner(),
            robot=robot_model(),
            collision_world=world(include_payload=False),
            payload=payload(),
        )


def test_coordinator_exposes_continuity_limits() -> None:
    """连续性阈值可注入，但默认是最严格的 1e-6 rad 端点重合。"""

    coordinator = _coordinator(RecordingPlanner(), payload=payload())
    assert coordinator.continuity_limits == JointContinuityLimits()
    with pytest.raises(ValueError):
        JointContinuityLimits(max_segment_gap_rad=-0.1)
