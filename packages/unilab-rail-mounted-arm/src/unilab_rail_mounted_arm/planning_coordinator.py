"""导轨机械臂的公共分相位规划协调器（rail-then-arm）。

设计来源：``docs/design/cosimulation/22-public-planning-layer.md`` §3.2/P6/P7。

本模块只依赖 ``unilab_robot_contracts``，不导入 MoveIt/ROS，也不执行任何硬件
动作。它把 ``ResolvedWorkCellPlan`` 的运动相位翻译成若干个独立的
``PlanRequest``，逐个调用注入的 ``MotionPlanner``：

1. ``rail_transfer_safe``：只规划机械臂到安全姿态，导轨锁在当前冻结值；
2. ``RAIL_MOVE``：只规划导轨到站点值，机械臂锁在安全姿态；
3. ``ARM_MOVE``：锁导轨在站点值，只规划机械臂。

关键不变量：

* 每个请求只允许一个轴组运动，**永远不把导轨并进关节向量做一次性 7 自由度
  联合规划**；
* 导轨值来自 ``ResolvedWorkCellPlan``/``ResolvedPointTarget``；缺失即失败关闭，
  绝不回落到 0；
* 每个成功相位都必须通过 ``PlanResult.validate_against``、运动轴锁检查和可选的
  逐路点碰撞校验；
* 任一相位失败，整个 ``WorkCellPlanResult`` 失败，顶层 ``waypoints`` 为空，
  已完成的相位轨迹只保留在相位明细中用于诊断，不拼接成可执行轨迹；
* 装配 ``payload`` 时，每个运动相位请求都携带同一个 ``PayloadAttachment``；
  规划器必须回显 payload identity/profile/attachment digest，否则整条计划
  ``invalid``（``payload_not_recorded``/``payload_identity_mismatch``）；
* 全部相位成功后再复核关节连续性：相邻段端点必须逐位相接、段内不得出现
  2π 整圈重标定，否则 ``invalid``（``joint_discontinuity``）；
* ``PlannedPhase``、``WorkCellPlanResult`` 是导轨机械臂包内的公共 dataclass，
  不扩展 contracts 五态合同。
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field, replace
from typing import Any, Protocol

from unilab_robot_contracts import (
    PLAN_STATUSES,
    AccessMotionPlanCompiler,
    ActionKind,
    CollisionWorld,
    JointContinuityLimits,
    MotionPlanner,
    PayloadAttachment,
    PlanConstraints,
    PlanGoal,
    PlanRequest,
    PlanResult,
    PlanStatus,
    PlanWaypoint,
    PlannerIdentity,
    RailAxis,
    ResolvedPointTarget,
    ResolvedWorkCellPlan,
    RobotModel,
    SafePoseConstraint,
    WorkCellPhaseKind,
    WorkCellPlanPhase,
    check_joint_continuity,
)

_SCHEMA = "unilab.rail-mounted-arm.workcell-plan/v1"
_ARM_KIND = WorkCellPhaseKind.ARM_MOVE.value
_RAIL_KIND = WorkCellPhaseKind.RAIL_MOVE.value
_END_EFFECTOR_KIND = WorkCellPhaseKind.END_EFFECTOR.value
_OBSERVE_KIND = WorkCellPhaseKind.OBSERVE.value
_MOTION_KINDS = frozenset({_ARM_KIND, _RAIL_KIND})
_KNOWN_KINDS = _MOTION_KINDS | {_END_EFFECTOR_KIND, _OBSERVE_KIND}
_Q_LOCK_TOL_RAD = 1e-6
_RAIL_LOCK_TOL_M = 1e-6
_DEFAULT_RAIL_TOLERANCE_M = 1e-4


class WaypointValidator(Protocol):
    """可选的逐路点场景碰撞校验端口；由独立场景检查实现。"""

    def __call__(
        self, request: PlanRequest, waypoint: PlanWaypoint
    ) -> tuple[bool, Mapping[str, Any]]:
        """返回 (是否有效, 诊断)；诊断必须是 JSON 可规范化映射。"""


def _canonical_json(value: object) -> str:
    """把值编码成稳定 JSON，供摘要与断言使用。"""

    try:
        return json.dumps(
            value,
            ensure_ascii=True,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        )
    except (TypeError, ValueError) as error:
        raise ValueError("WorkCell 计划结果必须可规范化为 JSON") from error


def _digest(value: object) -> str:
    """返回规范 JSON 的 SHA-256。"""

    return hashlib.sha256(_canonical_json(value).encode("utf-8")).hexdigest()


def _finite(value: object, name: str) -> float:
    """把值规范化为有限浮点数，布尔/字符串/NaN/Inf 一律拒绝。"""

    if isinstance(value, bool) or isinstance(value, (str, bytes)):
        raise ValueError(f"{name} 必须是有限数值")
    try:
        normalized = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} 必须是有限数值") from error
    if not math.isfinite(normalized):
        raise ValueError(f"{name} 必须是有限数值")
    return normalized


def _kind_of(phase: WorkCellPlanPhase) -> str:
    """把相位类型规范化为字符串值，兼容手工构造的 str。"""

    kind = phase.kind
    return kind.value if isinstance(kind, WorkCellPhaseKind) else str(kind)


def _q_close(
    left: Sequence[float], right: Sequence[float], tolerance: float
) -> bool:
    """返回两组关节值是否在容差内一致。"""

    return max(
        abs(float(a) - float(b)) for a, b in zip(left, right, strict=True)
    ) <= tolerance


def _rail_close(
    left: float | None, right: float | None, tolerance: float = _RAIL_LOCK_TOL_M
) -> bool:
    """返回两个导轨值是否在容差内一致；None 只在双方都缺省时一致。"""

    if left is None or right is None:
        return left is None and right is None
    return abs(float(left) - float(right)) <= tolerance


def _override_result(
    result: PlanResult, status: PlanStatus, extra: Mapping[str, Any]
) -> PlanResult:
    """保留原始轨迹与载荷回显，只改判状态与诊断；digest 由合同重新计算。"""

    return PlanResult(
        status=status,
        waypoints=result.waypoints,
        planner=result.planner,
        collision_checks=result.collision_checks,
        safe_pose_used=result.safe_pose_used,
        diagnostics={**dict(result.diagnostics), **extra},
        payload_ref=result.payload_ref,
        payload_profile_digest=result.payload_profile_digest,
        attachment_digest=result.attachment_digest,
    )


@dataclass(frozen=True, slots=True)
class PlannedPhase:
    """一个已处理相位：请求、原始结果、相对时间轨迹与判定状态。

    ``waypoints`` 的时间从该相位起点算起（0 开始）。只有成功相位才保留轨迹；
    失败相位只保留 ``result`` 供诊断，避免误拼接。
    """

    phase_id: str
    kind: str
    status: PlanStatus | None
    planned: bool
    start_q: tuple[float, ...] | None = None
    goal_q: tuple[float, ...] | None = None
    start_rail_m: float | None = None
    goal_rail_m: float | None = None
    request: PlanRequest | None = None
    result: PlanResult | None = None
    waypoints: tuple[PlanWaypoint, ...] = ()
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """拒绝空相位名、未知状态和错误值对象。"""

        if not isinstance(self.phase_id, str) or not self.phase_id.strip():
            raise ValueError("PlannedPhase.phase_id 不能为空")
        if self.status is not None and self.status not in PLAN_STATUSES:
            raise ValueError(f"未知 PlannedPhase.status: {self.status}")
        if self.request is not None and not isinstance(self.request, PlanRequest):
            raise TypeError("PlannedPhase.request 必须是 PlanRequest")
        if self.result is not None and not isinstance(self.result, PlanResult):
            raise TypeError("PlannedPhase.result 必须是 PlanResult")
        if isinstance(self.waypoints, (str, bytes)):
            raise TypeError("PlannedPhase.waypoints 必须是 PlanWaypoint 序列")
        waypoints = tuple(self.waypoints)
        if any(not isinstance(item, PlanWaypoint) for item in waypoints):
            raise TypeError("PlannedPhase.waypoints 只能包含 PlanWaypoint")
        object.__setattr__(self, "waypoints", waypoints)
        object.__setattr__(self, "diagnostics", dict(self.diagnostics))

    def as_mapping(self) -> dict[str, Any]:
        """返回报告用字段；轨迹只记数量，内容由 request/result 摘要绑定。"""

        return {
            "phase_id": self.phase_id,
            "kind": self.kind,
            "status": self.status,
            "planned": self.planned,
            "start_q": list(self.start_q) if self.start_q is not None else None,
            "goal_q": list(self.goal_q) if self.goal_q is not None else None,
            "start_rail_m": self.start_rail_m,
            "goal_rail_m": self.goal_rail_m,
            "waypoint_count": len(self.waypoints),
            "request_digest": self.request.digest if self.request is not None else None,
            "result_digest": self.result.digest if self.result is not None else None,
            "payload_ref": (
                self.result.payload_ref if self.result is not None else None
            ),
            "attachment_digest": (
                self.result.attachment_digest
                if self.result is not None
                else None
            ),
            "collision_checks": (
                self.result.collision_checks if self.result is not None else 0
            ),
            "diagnostics": dict(self.diagnostics),
        }


@dataclass(frozen=True, slots=True)
class WorkCellPlanResult:
    """分相位 WorkCell 规划的总结果。

    ``waypoints`` 是成功时按相位顺序合并出的绝对时间轨迹；失败时**必须为空**，
    各相位轨迹只保留在 ``phases`` 中用于诊断。``duration_s`` 由 ``waypoints``
    导出，避免时间字段与轨迹不一致。``digest`` 绑定源计划摘要与全部相位内容。
    """

    status: PlanStatus
    plan_ref: str
    action: ActionKind
    phases: tuple[PlannedPhase, ...]
    waypoints: tuple[PlanWaypoint, ...]
    planner: PlannerIdentity | None
    source_plan_digest: str | None
    failure_phase: str | None
    failure_reason: str | None
    diagnostics: Mapping[str, Any] = field(default_factory=dict)
    digest: str = ""
    payload_ref: str | None = None
    attachment_digest: str | None = None

    def __post_init__(self) -> None:
        """校验五态、失败不拼接轨迹、时间单调与摘要一致性。"""

        if self.status not in PLAN_STATUSES:
            raise ValueError(f"未知 WorkCellPlanResult.status: {self.status}")
        if not isinstance(self.plan_ref, str) or not self.plan_ref.strip():
            raise ValueError("WorkCellPlanResult.plan_ref 不能为空")
        if not isinstance(self.action, ActionKind):
            raise TypeError("WorkCellPlanResult.action 必须是 ActionKind")
        if isinstance(self.phases, (str, bytes)):
            raise TypeError("WorkCellPlanResult.phases 必须是 PlannedPhase 序列")
        phases = tuple(self.phases)
        if any(not isinstance(item, PlannedPhase) for item in phases):
            raise TypeError("WorkCellPlanResult.phases 只能包含 PlannedPhase")
        object.__setattr__(self, "phases", phases)
        if isinstance(self.waypoints, (str, bytes)):
            raise TypeError("WorkCellPlanResult.waypoints 必须是 PlanWaypoint 序列")
        waypoints = tuple(self.waypoints)
        if any(not isinstance(item, PlanWaypoint) for item in waypoints):
            raise TypeError("WorkCellPlanResult.waypoints 只能包含 PlanWaypoint")
        object.__setattr__(self, "waypoints", waypoints)
        if self.status == "success":
            if not waypoints:
                raise ValueError("成功 WorkCell 计划不得包含空轨迹")
            if self.failure_phase is not None:
                raise ValueError("成功 WorkCell 计划不得携带 failure_phase")
        else:
            if waypoints:
                raise ValueError(
                    "失败 WorkCell 计划不得携带拼接轨迹；各相位轨迹只保留在 phases"
                )
        if waypoints:
            dimensions = {len(item.q) for item in waypoints}
            if len(dimensions) != 1:
                raise ValueError("WorkCell 轨迹所有路点必须具有相同关节维度")
            times = [item.time_s for item in waypoints]
            if any(later < earlier for earlier, later in zip(times, times[1:])):
                raise ValueError("WorkCell 轨迹 waypoint.time_s 必须非递减")
        if self.planner is not None and not isinstance(self.planner, PlannerIdentity):
            raise TypeError("WorkCellPlanResult.planner 必须是 PlannerIdentity 或 None")
        if self.source_plan_digest is not None and not isinstance(
            self.source_plan_digest, str
        ):
            raise TypeError("WorkCellPlanResult.source_plan_digest 必须是字符串或 None")
        for name in ("failure_phase", "failure_reason"):
            value = getattr(self, name)
            if value is not None and not isinstance(value, str):
                raise TypeError(f"WorkCellPlanResult.{name} 必须是字符串或 None")
        declared = (self.payload_ref, self.attachment_digest)
        if any(value is not None for value in declared) and any(
            value is None for value in declared
        ):
            raise ValueError(
                "WorkCellPlanResult 的 payload_ref 与 attachment_digest 必须同时提供"
            )
        if self.payload_ref is not None:
            if not self.payload_ref.strip():
                raise ValueError("WorkCellPlanResult.payload_ref 不能为空")
            if (
                not isinstance(self.attachment_digest, str)
                or len(self.attachment_digest) != 64
                or any(
                    character not in "0123456789abcdef"
                    for character in self.attachment_digest
                )
            ):
                raise ValueError(
                    "WorkCellPlanResult.attachment_digest 必须是 SHA-256"
                )
        object.__setattr__(self, "diagnostics", dict(self.diagnostics))
        computed = _digest(self.as_mapping(include_digest=False))
        if self.digest and self.digest != computed:
            raise ValueError("WorkCellPlanResult.digest 与内容不一致")
        object.__setattr__(self, "digest", computed)

    @property
    def duration_s(self) -> float:
        """返回合并轨迹的总时长；失败结果没有可执行轨迹，恒为 0。"""

        return self.waypoints[-1].time_s if self.waypoints else 0.0

    @property
    def total_duration_s(self) -> float:
        """``duration_s`` 的显式别名。"""

        return self.duration_s

    def as_mapping(self, *, include_digest: bool = True) -> dict[str, Any]:
        """返回报告用规范映射；摘要计算时排除自身。"""

        result: dict[str, Any] = {
            "schema": _SCHEMA,
            "status": self.status,
            "plan_ref": self.plan_ref,
            "action": self.action.value,
            "source_plan_digest": self.source_plan_digest,
            "planner": self.planner.as_mapping() if self.planner is not None else None,
            "duration_s": self.duration_s,
            "failure_phase": self.failure_phase,
            "failure_reason": self.failure_reason,
            "phases": [phase.as_mapping() for phase in self.phases],
            "waypoints": [point.as_mapping() for point in self.waypoints],
            "diagnostics": dict(self.diagnostics),
        }
        if self.payload_ref is not None:
            result["payload_ref"] = self.payload_ref
            result["attachment_digest"] = self.attachment_digest
        if include_digest:
            result["digest"] = self.digest
        return result


@dataclass(frozen=True, slots=True)
class _Failure:
    """内部失败记录：失败相位、分类、原因与原始诊断。"""

    phase_id: str | None
    category: str
    reason: str
    diagnostics: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class _PhasePlan:
    """一个运动相位翻译出的独立请求与锁定状态。"""

    request: PlanRequest
    start_q: tuple[float, ...]
    goal_q: tuple[float, ...] | None
    start_rail_m: float | None
    goal_rail_m: float | None


@dataclass(frozen=True, slots=True)
class _PlanShape:
    """通过预检后的计划形态：运动相位、安全相位与站点导轨值。"""

    motion_phases: tuple[WorkCellPlanPhase, ...]
    safe_phase_id: str | None
    rail_move_phase_id: str | None
    effective_rail_m: float | None


class PlanningCoordinator:
    """把 ``ResolvedWorkCellPlan`` 相位翻译成 rail-then-arm 独立规划请求。"""

    def __init__(
        self,
        *,
        planner: MotionPlanner,
        robot: RobotModel,
        rail_axis: RailAxis | None = None,
        safe_pose: SafePoseConstraint | None = None,
        collision_world: CollisionWorld | None = None,
        constraints: PlanConstraints | None = None,
        compiler: AccessMotionPlanCompiler | None = None,
        policy_ref: str = "standard",
        timeout_s: float = 10.0,
        options: Mapping[str, Any] | None = None,
        waypoint_validator: WaypointValidator | None = None,
        payload: PayloadAttachment | None = None,
        continuity_limits: JointContinuityLimits | None = None,
    ) -> None:
        """冻结规划器、模型、导轨限位、安全姿态、可选载荷与连续性阈值。

        参数：``planner`` 为任意 ``MotionPlanner``；声明 ``rail_axis`` 时必须
        同时提供 ``safe_pose``（rail_transfer_safe 的目标）；``compiler`` 用于
        ``plan_workcell`` 直接消费 ``ResolvedPointTarget``；``payload`` 是本次
        WorkCell 计划全程携带的夹持载荷（必须已在其碰撞世界里，见
        ``PayloadAttachment.scene_problems``）。返回：无。异常：类型、装配、
        安全姿态或载荷/碰撞世界不一致时拒绝构造。
        """

        if not isinstance(planner, MotionPlanner):
            raise TypeError("planner 必须实现 MotionPlanner.plan")
        if not isinstance(robot, RobotModel):
            raise TypeError("robot 必须是 RobotModel")
        if rail_axis is not None:
            if not isinstance(rail_axis, RailAxis):
                raise TypeError("rail_axis 必须是 RailAxis 或 None")
            if rail_axis.axis_id in robot.joint_names:
                raise ValueError("rail_axis.axis_id 不得与机械臂关节同名")
            if safe_pose is None:
                raise ValueError(
                    "导轨装配必须提供安全姿态（rail_transfer_safe），不得缺省"
                )
        if safe_pose is not None:
            if not isinstance(safe_pose, SafePoseConstraint):
                raise TypeError("safe_pose 必须是 SafePoseConstraint 或 None")
            robot.validate_joint_positions(
                safe_pose.joint_positions, name="safe_pose.joint_positions"
            )
        if collision_world is None:
            collision_world = CollisionWorld()
        if not isinstance(collision_world, CollisionWorld):
            raise TypeError("collision_world 必须是 CollisionWorld")
        if constraints is None:
            constraints = PlanConstraints()
        if not isinstance(constraints, PlanConstraints):
            raise TypeError("constraints 必须是 PlanConstraints")
        if constraints.safe_pose is not None and safe_pose is not None:
            if tuple(constraints.safe_pose.joint_positions) != tuple(
                safe_pose.joint_positions
            ):
                raise ValueError("constraints.safe_pose 与注入的安全姿态不一致")
        if compiler is not None and not isinstance(compiler, AccessMotionPlanCompiler):
            raise TypeError("compiler 必须是 AccessMotionPlanCompiler 或 None")
        if not isinstance(policy_ref, str) or not policy_ref.strip():
            raise ValueError("policy_ref 不能为空")
        timeout = _finite(timeout_s, "timeout_s")
        if timeout <= 0.0:
            raise ValueError("timeout_s 必须为正")
        if options is None:
            options = {}
        if not isinstance(options, Mapping):
            raise TypeError("options 必须是映射")
        if waypoint_validator is not None and not callable(waypoint_validator):
            raise TypeError("waypoint_validator 必须可调用")
        if payload is not None and not isinstance(payload, PayloadAttachment):
            raise TypeError("payload 必须是 PayloadAttachment 或 None")
        problems = (
            payload.scene_problems(
                collision_world, flange_link=robot.flange_link
            )
            if payload is not None
            else ()
        )
        if problems:
            raise ValueError("；".join(problems))
        if continuity_limits is None:
            continuity_limits = JointContinuityLimits()
        if not isinstance(continuity_limits, JointContinuityLimits):
            raise TypeError("continuity_limits 必须是 JointContinuityLimits 或 None")
        self.planner = planner
        self.robot = robot
        self.rail_axis = rail_axis
        self.safe_pose = safe_pose
        self.collision_world = collision_world
        self.constraints = constraints
        self.compiler = compiler
        self.policy_ref = policy_ref
        self.timeout_s = timeout
        self.options = dict(options)
        self.waypoint_validator = waypoint_validator
        self.payload = payload
        self.continuity_limits = continuity_limits
        self.rail_tolerance_m = _finite(
            self.options.get("rail_goal_tolerance_m", _DEFAULT_RAIL_TOLERANCE_M),
            "options.rail_goal_tolerance_m",
        )
        if self.rail_tolerance_m < 0.0:
            raise ValueError("options.rail_goal_tolerance_m 不得为负")

    # ------------------------------------------------------------- 公共入口
    def plan_workcell(
        self,
        target: ResolvedPointTarget,
        *,
        action: ActionKind,
        current_rail_m: float | None,
        start_q: Sequence[float],
        goal_for: Mapping[str, PlanGoal] | Callable[[WorkCellPlanPhase], PlanGoal],
        policy_ref: str | None = None,
    ) -> WorkCellPlanResult:
        """先编译 ``ResolvedPointTarget``，再按相位规划整个 WorkCell 计划。"""

        if not isinstance(target, ResolvedPointTarget):
            raise TypeError("target 必须是 ResolvedPointTarget")
        if not isinstance(action, ActionKind):
            raise TypeError("action 必须是 ActionKind")
        if self.compiler is None:
            raise ValueError(
                "未注入 AccessMotionPlanCompiler，无法从 ResolvedPointTarget 编译计划"
            )
        has_rail = self.rail_axis is not None
        if not has_rail and target.rail_position_si is not None:
            raise ValueError("单机械臂协调器不得接收带站点导轨值的目标")
        compiled = self.compiler.compile(
            target,
            action=action,
            policy_ref=self.policy_ref if policy_ref is None else policy_ref,
            current_rail_position_si=current_rail_m,
            has_rail=has_rail,
        )
        return self.plan(
            compiled,
            start_q=start_q,
            start_rail_m=current_rail_m if has_rail else None,
            goal_for=goal_for,
        )

    def plan(
        self,
        plan: ResolvedWorkCellPlan,
        *,
        start_q: Sequence[float],
        start_rail_m: float | None,
        goal_for: Mapping[str, PlanGoal] | Callable[[WorkCellPlanPhase], PlanGoal],
    ) -> WorkCellPlanResult:
        """逐相位规划一个已编译 WorkCell 计划；失败不返回拼接轨迹。"""

        if not isinstance(plan, ResolvedWorkCellPlan):
            raise TypeError("plan 必须是 ResolvedWorkCellPlan")
        if not isinstance(plan.target_ref, str) or not plan.target_ref.strip():
            raise ValueError("ResolvedWorkCellPlan.target_ref 不能为空")
        if not isinstance(plan.action, ActionKind):
            raise TypeError("ResolvedWorkCellPlan.action 必须是 ActionKind")
        if not isinstance(plan.digest, str) or not plan.digest.strip():
            raise ValueError("ResolvedWorkCellPlan.digest 不能为空")
        if isinstance(plan.phases, (str, bytes)):
            raise TypeError("ResolvedWorkCellPlan.phases 必须是相位序列")
        phase_list = tuple(plan.phases)
        if not phase_list:
            raise ValueError("ResolvedWorkCellPlan 不得为空")
        if any(not isinstance(item, WorkCellPlanPhase) for item in phase_list):
            raise TypeError("ResolvedWorkCellPlan.phases 只能包含 WorkCellPlanPhase")
        if not callable(goal_for) and not isinstance(goal_for, Mapping):
            raise TypeError("goal_for 必须是映射或可调用对象")
        start_q = self.robot.validate_joint_positions(start_q, name="start_q")
        start_rail = self._validated_start_rail(start_rail_m)
        preflight = self._preflight(
            plan, start_q=start_q, start_rail_m=start_rail
        )
        if isinstance(preflight, _Failure):
            return self._failure_result(plan, preflight)
        return self._plan_shape(
            plan,
            preflight,
            phase_list=phase_list,
            start_q=start_q,
            start_rail_m=start_rail,
            goal_for=goal_for,
        )

    # ------------------------------------------------------------- 规划主体
    def _plan_shape(
        self,
        plan: ResolvedWorkCellPlan,
        shape: _PlanShape,
        *,
        phase_list: tuple[WorkCellPlanPhase, ...],
        start_q: tuple[float, ...],
        start_rail_m: float | None,
        goal_for: Mapping[str, PlanGoal] | Callable[[WorkCellPlanPhase], PlanGoal],
    ) -> WorkCellPlanResult:
        """逐相位构造请求、调用规划器并合并成功轨迹。"""

        current_q: tuple[float, ...] = start_q
        current_rail = start_rail_m
        identity: PlannerIdentity | None = None
        merged: list[PlanWaypoint] = []
        offset = 0.0
        entries: list[PlannedPhase] = []
        failure: _Failure | None = None
        status: PlanStatus = "success"

        for index, phase in enumerate(phase_list):
            kind = _kind_of(phase)
            if kind not in _MOTION_KINDS:
                entries.append(self._unplanned_entry(phase))
                continue
            phase_plan = self._request_for_phase(
                phase,
                shape,
                current_q=current_q,
                current_rail=current_rail,
                goal_for=goal_for,
            )
            if isinstance(phase_plan, _Failure):
                failure = phase_plan
                status = "invalid"
                entries.append(self._rejected_entry(phase, phase_plan))
                entries.extend(
                    self._unplanned_entry(remaining)
                    for remaining in phase_list[index + 1 :]
                )
                break
            request = phase_plan.request
            try:
                result = self.planner.plan(request)
            except Exception as exc:  # noqa: BLE001 - 后端异常必须失败关闭
                detail = f"{type(exc).__name__}: {exc}"
                failure = _Failure(
                    phase.phase_id,
                    "planner_exception",
                    f"相位 {phase.phase_id} 规划器异常: {detail}",
                    {"error": detail},
                )
                status = "invalid"
                entries.append(self._rejected_entry(phase, failure))
                entries.extend(
                    self._unplanned_entry(remaining)
                    for remaining in phase_list[index + 1 :]
                )
                break
            if not isinstance(result, PlanResult):
                detail = f"规划器返回 {type(result).__name__}，不是 PlanResult"
                failure = _Failure(
                    phase.phase_id,
                    "planner_result_invalid",
                    f"相位 {phase.phase_id} 结果非法: {detail}",
                    {"error": detail},
                )
                status = "invalid"
                entries.append(self._rejected_entry(phase, failure))
                entries.extend(
                    self._unplanned_entry(remaining)
                    for remaining in phase_list[index + 1 :]
                )
                break
            result, failure = self._adjudicate(
                phase.phase_id, request, result, identity
            )
            if identity is None:
                identity = result.planner
            if failure is not None:
                status = result.status
                entries.append(
                    self._phase_entry(
                        phase,
                        phase_plan,
                        result,
                        waypoints=(),
                        diagnostics={
                            "failure_category": failure.category,
                            "error": failure.reason,
                        },
                    )
                )
                entries.extend(
                    self._unplanned_entry(remaining)
                    for remaining in phase_list[index + 1 :]
                )
                break

            shifted = tuple(
                PlanWaypoint(
                    point.q, point.rail_m, offset + point.time_s, point.source
                )
                for point in result.waypoints
            )
            if shifted:
                offset = shifted[-1].time_s
                merged.extend(shifted)
                current_q = tuple(shifted[-1].q)
                current_rail = float(shifted[-1].rail_m)
            entries.append(
                self._phase_entry(
                    phase, phase_plan, result, waypoints=tuple(result.waypoints)
                )
            )

        if failure is None and not merged:
            failure = _Failure(
                None,
                "empty_phased_trajectory",
                "WorkCell 计划没有产生可执行轨迹",
                {},
            )
            status = "invalid"

        if failure is None:
            continuity = self._continuity_failure(entries)
            if continuity is not None:
                failure, status = continuity, "invalid"

        return self._result(
            plan,
            shape=shape,
            status=status,
            entries=tuple(entries),
            merged=tuple(merged) if failure is None else (),
            planner=identity,
            failure=failure,
        )

    def _continuity_failure(
        self, entries: Sequence[PlannedPhase]
    ) -> _Failure | None:
        """按执行顺序检查已规划相位的关节连续性；不连续即整体 invalid。

        协调器逐相位把上一段末路点当作下一段 ``start_q``，所以段间端点本来就
        应该相接；这里仍然显式复核一次，防止换实现后出现整圈重标定，并检查
        段内单步 2π 跳变（动力学臂无法跟随的“腕部整圈”）。
        """

        planned = [
            entry for entry in entries if entry.planned and entry.result is not None
        ]
        if not planned:
            return None
        results = tuple(entry.result for entry in planned)
        assert all(result is not None for result in results)
        report = check_joint_continuity(
            results, limits=self.continuity_limits  # type: ignore[arg-type]
        )
        if report.ok:
            return None
        first = dict(report.violations[0])
        index = first.get("segment_index")
        phase_id = (
            planned[index].phase_id
            if isinstance(index, int) and 0 <= index < len(planned)
            else None
        )
        return _Failure(
            phase_id,
            "joint_discontinuity",
            f"WorkCell 轨迹关节不连续（{first.get('kind')}，段 {index}）",
            {"joint_continuity": report.as_mapping()},
        )

    def _preflight(
        self,
        plan: ResolvedWorkCellPlan,
        *,
        start_q: tuple[float, ...],
        start_rail_m: float | None,
    ) -> _PlanShape | _Failure:
        """在调用规划器前检查相位顺序、导轨值和站点锁定。"""

        kinds = [_kind_of(phase) for phase in plan.phases]
        for phase, kind in zip(plan.phases, kinds, strict=True):
            if kind not in _KNOWN_KINDS:
                return _Failure(
                    phase.phase_id,
                    "phase_kind_invalid",
                    f"未知相位类型: {kind!r}",
                )
        motion = tuple(
            phase for phase, kind in zip(plan.phases, kinds, strict=True)
            if kind in _MOTION_KINDS
        )
        if not motion:
            return _Failure(
                None, "no_motion_phases", "WorkCell 计划至少需要一个运动相位"
            )

        if self.rail_axis is None:
            if plan.effective_rail_position_si is not None:
                return _Failure(
                    motion[0].phase_id,
                    "rail_axis_missing",
                    "计划声明了站点导轨值，但协调器未装配导轨",
                )
            for phase, kind in zip(plan.phases, kinds, strict=True):
                if kind in _MOTION_KINDS and phase.rail_position_si is not None:
                    return _Failure(
                        phase.phase_id,
                        "rail_axis_missing",
                        "无导轨装配时相位不得携带导轨值",
                    )
            return _PlanShape(motion, None, None, None)

        assert start_rail_m is not None
        try:
            effective = _finite(
                plan.effective_rail_position_si, "effective_rail_position_si"
            )
        except (TypeError, ValueError):
            return _Failure(
                motion[0].phase_id,
                "rail_position_missing",
                "导轨计划必须显式给出站点导轨值，不得默认 0",
            )
        if not self.rail_axis.contains(effective):
            return _Failure(
                motion[0].phase_id,
                "rail_target_out_of_limits",
                f"站点导轨值 {effective} 超出限位",
            )
        for phase, kind in zip(plan.phases, kinds, strict=True):
            if kind not in _MOTION_KINDS:
                continue
            if phase.rail_position_si is None:
                return _Failure(
                    phase.phase_id,
                    "rail_position_missing",
                    f"相位 {phase.phase_id} 缺少站点导轨值，不得默认 0",
                )
            try:
                phase_rail = _finite(
                    phase.rail_position_si, f"{phase.phase_id}.rail_position_si"
                )
            except (TypeError, ValueError):
                return _Failure(
                    phase.phase_id,
                    "rail_position_missing",
                    f"相位 {phase.phase_id} 的站点导轨值不是有限数",
                )
            if abs(phase_rail - effective) > self.rail_tolerance_m:
                return _Failure(
                    phase.phase_id,
                    "rail_position_inconsistent",
                    f"相位 {phase.phase_id} 导轨值 {phase_rail} 与站点值 "
                    f"{effective} 不一致",
                )

        rail_moves = tuple(
            phase for phase, kind in zip(plan.phases, kinds, strict=True)
            if kind == _RAIL_KIND
        )
        if len(rail_moves) > 1:
            return _Failure(
                rail_moves[1].phase_id,
                "phase_order_invalid",
                "同一 WorkCell 计划只允许一个 RAIL_MOVE 相位",
            )
        if not rail_moves:
            if not _rail_close(start_rail_m, effective, self.rail_tolerance_m):
                return _Failure(
                    motion[0].phase_id,
                    "rail_not_settled",
                    f"机械臂运动前导轨必须已由 RAIL_MOVE 到位: "
                    f"{start_rail_m} != {effective}",
                )
            return _PlanShape(motion, None, None, effective)

        rail_index = next(
            index for index, phase in enumerate(motion) if phase is rail_moves[0]
        )
        pre_arm = tuple(
            phase for phase in motion[:rail_index] if _kind_of(phase) == _ARM_KIND
        )
        if len(pre_arm) > 1:
            return _Failure(
                pre_arm[1].phase_id,
                "phase_order_invalid",
                "导轨换位前只允许一个 rail_transfer_safe 机械臂相位",
            )
        safe_phase_id: str | None = None
        if pre_arm:
            safe_phase_id = pre_arm[0].phase_id
        else:
            assert self.safe_pose is not None
            if not _q_close(
                start_q,
                self.safe_pose.joint_positions,
                self.safe_pose.tolerance_rad,
            ):
                return _Failure(
                    rail_moves[0].phase_id,
                    "rail_move_requires_safe_pose",
                    "RAIL_MOVE 前机械臂必须已在 rail_transfer_safe 安全姿态",
                )
        return _PlanShape(
            motion, safe_phase_id, rail_moves[0].phase_id, effective
        )

    def _request_for_phase(
        self,
        phase: WorkCellPlanPhase,
        shape: _PlanShape,
        *,
        current_q: tuple[float, ...],
        current_rail: float | None,
        goal_for: Mapping[str, PlanGoal] | Callable[[WorkCellPlanPhase], PlanGoal],
    ) -> _PhasePlan | _Failure:
        """把一个相位翻译成只允许一个轴组运动的独立 ``PlanRequest``。"""

        kind = _kind_of(phase)
        goal_q: tuple[float, ...] | None = None
        if kind == _RAIL_KIND:
            assert self.safe_pose is not None
            if not _q_close(
                current_q,
                self.safe_pose.joint_positions,
                self.safe_pose.tolerance_rad,
            ):
                return _Failure(
                    phase.phase_id,
                    "rail_move_requires_safe_pose",
                    "RAIL_MOVE 前机械臂必须已在 rail_transfer_safe 安全姿态",
                )
            try:
                target_rail = _finite(
                    phase.rail_position_si, f"{phase.phase_id}.rail_position_si"
                )
            except (TypeError, ValueError):
                return _Failure(
                    phase.phase_id,
                    "rail_position_missing",
                    "RAIL_MOVE 缺少显式站点导轨值，不得默认 0",
                )
            assert self.rail_axis is not None
            if not self.rail_axis.contains(target_rail):
                return _Failure(
                    phase.phase_id,
                    "rail_target_out_of_limits",
                    f"RAIL_MOVE 目标 {target_rail} 超出导轨限位",
                )
            goal = PlanGoal.from_joint(current_q)
            goal_q = tuple(current_q)
            start_rail = current_rail
            goal_rail = target_rail
        else:
            if phase.phase_id == shape.safe_phase_id:
                assert self.safe_pose is not None
                goal = PlanGoal.from_joint(self.safe_pose.joint_positions)
                goal_q = tuple(self.safe_pose.joint_positions)
            else:
                if self.rail_axis is not None:
                    assert shape.effective_rail_m is not None
                    if not _rail_close(
                        current_rail, shape.effective_rail_m, self.rail_tolerance_m
                    ):
                        return _Failure(
                            phase.phase_id,
                            "rail_not_settled",
                            f"ARM_MOVE 前导轨未到位: "
                            f"{current_rail} != {shape.effective_rail_m}",
                        )
                resolved = self._resolve_goal(goal_for, phase)
                if isinstance(resolved, _Failure):
                    return resolved
                goal = resolved
                goal_q = (
                    tuple(goal.joint_positions)
                    if goal.kind == "joint" and goal.joint_positions is not None
                    else None
                )
            start_rail = current_rail
            goal_rail = current_rail

        request_kwargs: dict[str, Any] = {
            "robot": self.robot,
            "start_q": current_q,
            "goal": goal,
            "collision_world": self.collision_world,
            "constraints": self.constraints,
            "timeout_s": self.timeout_s,
            "options": dict(self.options),
        }
        if self.payload is not None:
            # 载荷必须逐相位进入规划器：附件、携带集合、物料身份与 Profile
            # 四处一致，规划器才有机会把它装进自己的规划场景。
            request_kwargs.update(
                attachments=(self.payload.attachment,),
                carried_body_ids=(self.payload.body_id,),
                held_material=self.payload.payload_ref,
                payload=self.payload,
            )
        if self.rail_axis is not None:
            request_kwargs.update(
                rail_axis=self.rail_axis,
                start_rail_m=start_rail,
                goal_rail_m=goal_rail,
            )
        try:
            request = PlanRequest(**request_kwargs)
        except (TypeError, ValueError) as exc:
            return _Failure(
                phase.phase_id,
                "phase_request_invalid",
                f"相位 {phase.phase_id} 请求非法: {exc}",
                {"error": str(exc)},
            )
        return _PhasePlan(
            request, tuple(current_q), goal_q, start_rail, goal_rail
        )

    def _resolve_goal(
        self,
        goal_for: Mapping[str, PlanGoal] | Callable[[WorkCellPlanPhase], PlanGoal],
        phase: WorkCellPlanPhase,
    ) -> PlanGoal | _Failure:
        """按 target_ref 或回调解析关节/位姿目标；缺失即失败关闭。"""

        try:
            goal = (
                goal_for(phase)
                if callable(goal_for)
                else goal_for[phase.target_ref]
            )
        except Exception as exc:  # noqa: BLE001 - 缺键或回调异常都失败关闭
            return _Failure(
                phase.phase_id,
                "goal_unresolved",
                f"无法解析相位 {phase.phase_id} 的目标 "
                f"{phase.target_ref!r}: {type(exc).__name__}: {exc}",
            )
        if not isinstance(goal, PlanGoal):
            return _Failure(
                phase.phase_id,
                "goal_unresolved",
                f"相位 {phase.phase_id} 的目标必须是 PlanGoal，"
                f"实际 {type(goal).__name__}",
            )
        return goal

    def _adjudicate(
        self,
        phase_id: str,
        request: PlanRequest,
        result: PlanResult,
        identity: PlannerIdentity | None,
    ) -> tuple[PlanResult, _Failure | None]:
        """复核一段结果：合同校验、轴锁、碰撞与规划器身份。"""

        if result.status != "success":
            category = str(
                result.diagnostics.get("failure_category")
                or f"planner_{result.status}"
            )
            return result, _Failure(
                phase_id,
                category,
                f"相位 {phase_id} 规划失败: {result.status}",
                dict(result.diagnostics),
            )
        payload_problem = self._payload_problem(request, result)
        if payload_problem is not None:
            overridden = _override_result(result, "invalid", payload_problem)
            return overridden, _Failure(
                phase_id,
                str(payload_problem["failure_category"]),
                f"相位 {phase_id} 载荷回显不合法: {payload_problem.get('detail', '')}",
                dict(overridden.diagnostics),
            )
        try:
            result.validate_against(request)
        except (TypeError, ValueError) as exc:
            overridden = _override_result(
                result,
                "invalid",
                {
                    "failure_category": "phase_result_inconsistent",
                    "error": str(exc),
                },
            )
            return overridden, _Failure(
                phase_id,
                "phase_result_inconsistent",
                f"相位 {phase_id} 结果与请求不一致: {exc}",
                dict(overridden.diagnostics),
            )
        violation = self._lock_violation(request, result)
        if violation is not None:
            overridden = _override_result(result, "invalid", violation)
            return overridden, _Failure(
                phase_id,
                "rail_lock_violation",
                f"相位 {phase_id} 违反运动轴锁: {violation.get('detail', '')}",
                dict(overridden.diagnostics),
            )
        if self.waypoint_validator is not None:
            for index, waypoint in enumerate(result.waypoints):
                try:
                    valid, diagnostics = self.waypoint_validator(request, waypoint)
                except Exception as exc:  # noqa: BLE001 - 校验器异常失败关闭
                    detail = f"{type(exc).__name__}: {exc}"
                    overridden = _override_result(
                        result,
                        "invalid",
                        {
                            "failure_category": "waypoint_validator_error",
                            "waypoint_index": index,
                            "error": detail,
                        },
                    )
                    return overridden, _Failure(
                        phase_id,
                        "waypoint_validator_error",
                        f"相位 {phase_id} 路点 {index} 碰撞校验异常: {detail}",
                        dict(overridden.diagnostics),
                    )
                if not valid:
                    overridden = _override_result(
                        result,
                        "collision",
                        {
                            "failure_category": "waypoint_collision",
                            "waypoint_index": index,
                            "validator": dict(diagnostics),
                        },
                    )
                    return overridden, _Failure(
                        phase_id,
                        "waypoint_collision",
                        f"相位 {phase_id} 路点 {index} 未通过碰撞校验",
                        dict(overridden.diagnostics),
                    )
        if identity is not None and result.planner != identity:
            overridden = _override_result(
                result,
                "invalid",
                {
                    "failure_category": "planner_identity_mismatch",
                    "expected_planner": identity.as_mapping(),
                    "actual_planner": result.planner.as_mapping(),
                },
            )
            return overridden, _Failure(
                phase_id,
                "planner_identity_mismatch",
                "同一 WorkCell 计划内规划器身份发生变化",
                dict(overridden.diagnostics),
            )
        return result, None

    def _payload_problem(
        self, request: PlanRequest, result: PlanResult
    ) -> dict[str, Any] | None:
        """核对规划器是否回显了请求携带的载荷身份；返回失败诊断或 ``None``。

        规划器没有回显 = 它没有声明把载荷装进自己的规划场景；协调器不会
        用“请求里有载荷”替它作证，而是让整个计划失败关闭。规划器回显了
        请求未声明的载荷同样拒绝（身份必须双向一致）。
        """

        expected = request.payload
        if expected is None:
            if (
                result.payload_ref is not None
                or result.payload_profile_digest is not None
                or result.attachment_digest is not None
            ):
                return {
                    "failure_category": "payload_identity_mismatch",
                    "detail": "规划器回显了请求未声明的 payload 身份",
                    "actual_payload_ref": result.payload_ref,
                }
            return None
        if result.payload_ref is None:
            return {
                "failure_category": "payload_not_recorded",
                "detail": "规划器未回显请求携带的 payload identity/profile/digest",
                "expected_payload_ref": expected.payload_ref,
                "expected_attachment_digest": expected.digest,
            }
        if result.payload_ref != expected.payload_ref:
            return {
                "failure_category": "payload_identity_mismatch",
                "detail": "规划器回显的 payload identity 与请求不一致",
                "expected_payload_ref": expected.payload_ref,
                "actual_payload_ref": result.payload_ref,
            }
        if result.payload_profile_digest != expected.profile_digest:
            return {
                "failure_category": "payload_identity_mismatch",
                "detail": "规划器回显的 payload profile digest 与请求不一致",
                "expected_profile_digest": expected.profile_digest,
                "actual_profile_digest": result.payload_profile_digest,
            }
        if result.attachment_digest != expected.digest:
            return {
                "failure_category": "payload_identity_mismatch",
                "detail": "规划器回显的 attachment digest 与请求不一致",
                "expected_attachment_digest": expected.digest,
                "actual_attachment_digest": result.attachment_digest,
            }
        return None

    def _lock_violation(
        self, request: PlanRequest, result: PlanResult
    ) -> dict[str, Any] | None:
        """检查段内未运动轴是否真的保持锁定；返回首个违规路点。"""

        if request.rail_axis is None:
            return None
        if request.rail_moves:
            locked_q = request.start_q
            for index, waypoint in enumerate(result.waypoints):
                if not _q_close(waypoint.q, locked_q, _Q_LOCK_TOL_RAD):
                    return {
                        "failure_category": "rail_lock_violation",
                        "detail": "RAIL_MOVE 期间机械臂离开安全姿态",
                        "locked_axis": "arm",
                        "waypoint_index": index,
                        "expected_q": list(locked_q),
                        "actual_q": list(waypoint.q),
                    }
            return None
        assert request.start_rail_m is not None
        locked_rail = float(request.start_rail_m)
        for index, waypoint in enumerate(result.waypoints):
            if abs(waypoint.rail_m - locked_rail) > _RAIL_LOCK_TOL_M:
                return {
                    "failure_category": "rail_lock_violation",
                    "detail": "ARM_MOVE 期间导轨离开站点值",
                    "locked_axis": "rail",
                    "waypoint_index": index,
                    "expected_rail_m": locked_rail,
                    "actual_rail_m": waypoint.rail_m,
                }
        return None

    # ------------------------------------------------------------- 结果装配
    def _validated_start_rail(self, start_rail_m: float | None) -> float | None:
        """校验冻结的当前导轨值；声明导轨时绝不默认 0。"""

        if self.rail_axis is None:
            if start_rail_m is not None:
                raise ValueError("无导轨装配时不得提供 start_rail_m")
            return None
        if start_rail_m is None:
            raise ValueError(
                "导轨装配必须提供显式 start_rail_m（冻结当前导轨），不得默认 0"
            )
        value = _finite(start_rail_m, "start_rail_m")
        if not self.rail_axis.contains(value):
            raise ValueError("start_rail_m 超出导轨限位")
        return value

    def _base_diagnostics(
        self, plan: ResolvedWorkCellPlan, shape: _PlanShape | None = None
    ) -> dict[str, Any]:
        """返回与具体失败无关的计划身份/策略字段。"""

        diagnostics: dict[str, Any] = {
            "strategy": (
                "rail_then_arm" if self.rail_axis is not None else "arm_only"
            ),
            "joint_planning": "per_phase_single_axis_group",
            "robot_model_digest": self.robot.digest,
            "rail_axis": (
                self.rail_axis.as_mapping() if self.rail_axis is not None else None
            ),
            "source_plan_digest": plan.digest,
            "phase_count": len(plan.phases),
        }
        if self.safe_pose is not None:
            diagnostics["safe_pose"] = list(self.safe_pose.joint_positions)
        diagnostics["payload"] = (
            None
            if self.payload is None
            else {
                "payload_ref": self.payload.payload_ref,
                "profile_digest": self.payload.profile_digest,
                "attachment_digest": self.payload.digest,
                "body_id": self.payload.body_id,
                "link_name": self.payload.attachment.link_name,
            }
        )
        if shape is not None:
            diagnostics["motion_phase_count"] = len(shape.motion_phases)
            diagnostics["safe_phase_id"] = shape.safe_phase_id
            diagnostics["rail_move_phase_id"] = shape.rail_move_phase_id
            diagnostics["effective_rail_m"] = shape.effective_rail_m
        return diagnostics

    def _result(
        self,
        plan: ResolvedWorkCellPlan,
        *,
        shape: _PlanShape,
        status: PlanStatus,
        entries: tuple[PlannedPhase, ...],
        merged: tuple[PlanWaypoint, ...],
        planner: PlannerIdentity | None,
        failure: _Failure | None,
    ) -> WorkCellPlanResult:
        """把相位明细、合并轨迹和失败原因收敛为一个总结果。"""

        diagnostics = self._base_diagnostics(plan, shape)
        if failure is not None:
            diagnostics["failure_category"] = failure.category
            diagnostics["failure"] = {
                "phase_id": failure.phase_id,
                "status": status,
                "category": failure.category,
                "reason": failure.reason,
                "planner_diagnostics": dict(failure.diagnostics),
            }
            continuity = failure.diagnostics.get("joint_continuity")
            if continuity is not None:
                # 关节连续性是整条链的判据（不是某个相位），单独放在顶层。
                diagnostics["joint_continuity"] = dict(continuity)
        return WorkCellPlanResult(
            status=status,
            plan_ref=plan.target_ref,
            action=plan.action,
            phases=entries,
            waypoints=merged,
            planner=planner,
            source_plan_digest=plan.digest,
            failure_phase=failure.phase_id if failure is not None else None,
            failure_reason=failure.reason if failure is not None else None,
            diagnostics=diagnostics,
            payload_ref=(
                self.payload.payload_ref if self.payload is not None else None
            ),
            attachment_digest=(
                self.payload.digest if self.payload is not None else None
            ),
        )

    def _failure_result(
        self, plan: ResolvedWorkCellPlan, failure: _Failure
    ) -> WorkCellPlanResult:
        """预检失败：不调用规划器，所有相位都标记为未规划。"""

        entries: list[PlannedPhase] = []
        for phase in plan.phases:
            if failure.phase_id is not None and phase.phase_id == failure.phase_id:
                entries.append(self._rejected_entry(phase, failure))
            else:
                entries.append(self._unplanned_entry(phase))
        diagnostics = self._base_diagnostics(plan)
        diagnostics["failure_category"] = failure.category
        diagnostics["failure"] = {
            "phase_id": failure.phase_id,
            "status": "invalid",
            "category": failure.category,
            "reason": failure.reason,
            "planner_diagnostics": dict(failure.diagnostics),
        }
        return WorkCellPlanResult(
            status="invalid",
            plan_ref=plan.target_ref,
            action=plan.action,
            phases=tuple(entries),
            waypoints=(),
            planner=None,
            source_plan_digest=plan.digest,
            failure_phase=failure.phase_id,
            failure_reason=failure.reason,
            diagnostics=diagnostics,
            payload_ref=(
                self.payload.payload_ref if self.payload is not None else None
            ),
            attachment_digest=(
                self.payload.digest if self.payload is not None else None
            ),
        )

    @staticmethod
    def _unplanned_entry(phase: WorkCellPlanPhase) -> PlannedPhase:
        """把非运动相位或未执行相位记入明细，不产生请求/轨迹。"""

        return PlannedPhase(
            phase_id=phase.phase_id,
            kind=_kind_of(phase),
            status=None,
            planned=False,
            goal_rail_m=phase.rail_position_si,
        )

    @staticmethod
    def _rejected_entry(
        phase: WorkCellPlanPhase, failure: _Failure
    ) -> PlannedPhase:
        """记录未能构造请求或已被预检拒绝的相位。"""

        return PlannedPhase(
            phase_id=phase.phase_id,
            kind=_kind_of(phase),
            status="invalid",
            planned=False,
            goal_rail_m=phase.rail_position_si,
            diagnostics={
                "failure_category": failure.category,
                "error": failure.reason,
            },
        )

    @staticmethod
    def _phase_entry(
        phase: WorkCellPlanPhase,
        phase_plan: _PhasePlan,
        result: PlanResult,
        *,
        waypoints: tuple[PlanWaypoint, ...],
        diagnostics: Mapping[str, Any] | None = None,
    ) -> PlannedPhase:
        """把已调用规划器的相位收敛为明细项。"""

        return PlannedPhase(
            phase_id=phase.phase_id,
            kind=_kind_of(phase),
            status=result.status,
            planned=True,
            start_q=phase_plan.start_q,
            goal_q=phase_plan.goal_q,
            start_rail_m=phase_plan.start_rail_m,
            goal_rail_m=phase_plan.goal_rail_m,
            request=phase_plan.request,
            result=result,
            waypoints=waypoints,
            diagnostics=(
                dict(diagnostics)
                if diagnostics is not None
                else {"collision_checks": result.collision_checks}
            ),
        )


__all__ = [
    "PlannedPhase",
    "PlanningCoordinator",
    "WaypointValidator",
    "WorkCellPlanResult",
]
