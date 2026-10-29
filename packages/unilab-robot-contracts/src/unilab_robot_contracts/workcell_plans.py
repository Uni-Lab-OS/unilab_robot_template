"""把 PointSet、MotionProfile 和动作模式编译成不可变 WorkCell 计划。"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum

from .actions import ActionKind
from .motion_profiles import MotionProfileCatalog
from .point_set_types import ResolvedPointTarget
from .point_set_validation import stable_digest


class WorkCellPhaseKind(str, Enum):
    """导轨机械臂组合执行器理解的有限阶段类型。"""

    ARM_MOVE = "arm_move"
    RAIL_MOVE = "rail_move"
    END_EFFECTOR = "end_effector"
    OBSERVE = "observe"


@dataclass(frozen=True, slots=True)
class WorkCellPlanPhase:
    """一项完全解析且不可被 Workflow 改写的执行阶段。"""

    phase_id: str
    kind: WorkCellPhaseKind
    target_ref: str
    profile_ref: str | None
    rail_position_si: float | None
    payload_state: str


@dataclass(frozen=True, slots=True)
class ResolvedWorkCellPlan:
    """一个上层通用动作对应的确定性组合运动计划。"""

    target_ref: str
    action: ActionKind
    effective_rail_position_si: float | None
    phases: tuple[WorkCellPlanPhase, ...]
    digest: str


class AccessMotionPlanCompiler:
    """编译标准 pick/place 接近块并强制导轨先到位、全块不变。"""

    def __init__(
        self,
        *,
        profiles: MotionProfileCatalog,
        rail_transfer_safe_ref: str = "global.arm.rail_transfer_safe",
    ) -> None:
        """冻结运动策略目录和导轨换位安全姿态。"""

        self.profiles = profiles
        self.rail_transfer_safe_ref = rail_transfer_safe_ref

    def compile(
        self,
        target: ResolvedPointTarget,
        *,
        action: ActionKind,
        policy_ref: str,
        current_rail_position_si: float | None,
        has_rail: bool,
    ) -> ResolvedWorkCellPlan:
        """生成 rail barrier + D9-1S 序列，不允许导轨在块内变化。

        参数：已解析点位、typed 动作、策略、命令冻结时导轨位置和装配形态。
        返回：完全解析计划。异常：缺失接近块、导轨初态或不支持动作时关闭失败。
        """

        if action not in {ActionKind.PICK, ActionKind.PLACE}:
            raise ValueError("AccessMotionBlock v1 当前只编译标准 pick/place")
        if target.access_block is None:
            raise ValueError(f"{target.target_ref} 未声明 AccessMotionBlock")
        try:
            policy = self.profiles.access[policy_ref]
        except KeyError as exc:
            raise ValueError(f"缺少 AccessMotionPolicy: {policy_ref}") from exc
        effective_rail = self._effective_rail(
            target,
            current_rail_position_si=current_rail_position_si,
            has_rail=has_rail,
        )
        phases: list[WorkCellPlanPhase] = []
        if has_rail and effective_rail != current_rail_position_si:
            if policy.rail_profile_ref is None:
                raise ValueError("导轨换位必须显式引用 RailMotionProfile")
            phases.extend(
                [
                    _arm_phase(
                        "rail-transfer-safe",
                        self.rail_transfer_safe_ref,
                        policy.transit_profile_ref,
                        effective_rail,
                        "unchanged",
                    ),
                    WorkCellPlanPhase(
                        "rail-move-and-settle",
                        WorkCellPhaseKind.RAIL_MOVE,
                        target.target_ref,
                        policy.rail_profile_ref,
                        effective_rail,
                        "unchanged",
                    ),
                ]
            )
        block = target.access_block
        phases.extend(
            _arm_phase(
                f"transit-in-{index + 1}",
                target_ref,
                policy.transit_profile_ref,
                effective_rail,
                "empty" if action is ActionKind.PICK else "loaded",
            )
            for index, target_ref in enumerate(block.transit_in)
        )
        inbound_state = "empty" if action is ActionKind.PICK else "loaded"
        outbound_state = "loaded" if action is ActionKind.PICK else "empty"
        phases.extend(
            [
                _arm_phase("entry", block.entry_target_ref, policy.entry_profile_ref, effective_rail, inbound_state),
                _arm_phase("approach", block.approach_target_ref, policy.approach_profile_ref, effective_rail, inbound_state),
                _arm_phase("interaction", block.interaction_target_ref, policy.interaction_profile_ref, effective_rail, inbound_state),
                WorkCellPlanPhase(
                    action.value,
                    WorkCellPhaseKind.END_EFFECTOR,
                    f"end_effector.{ 'grip' if action is ActionKind.PICK else 'release' }",
                    None,
                    effective_rail,
                    outbound_state,
                ),
                WorkCellPlanPhase(
                    "observe-payload",
                    WorkCellPhaseKind.OBSERVE,
                    f"end_effector.payload.{outbound_state}",
                    None,
                    effective_rail,
                    outbound_state,
                ),
                _arm_phase("retract-approach", block.approach_target_ref, policy.approach_profile_ref, effective_rail, outbound_state),
                _arm_phase("retract-entry", block.entry_target_ref, policy.entry_profile_ref, effective_rail, outbound_state),
            ]
        )
        phases.extend(
            _arm_phase(
                f"transit-out-{index + 1}",
                target_ref,
                policy.transit_profile_ref,
                effective_rail,
                outbound_state,
            )
            for index, target_ref in enumerate(block.transit_out)
        )
        payload = {
            "target_ref": target.target_ref,
            "target_digest": target.resolved_digest,
            "action": action.value,
            "profiles_revision": self.profiles.revision,
            "policy_ref": policy.policy_ref,
            "effective_rail_position_si": effective_rail,
            "phases": [asdict(phase) for phase in phases],
        }
        return ResolvedWorkCellPlan(
            target.target_ref,
            action,
            effective_rail,
            tuple(phases),
            stable_digest(payload),
        )

    @staticmethod
    def _effective_rail(
        target: ResolvedPointTarget,
        *,
        current_rail_position_si: float | None,
        has_rail: bool,
    ) -> float | None:
        """落实“缺省 rail=保持冻结时当前位置”，禁止把它解释为自由轴。"""

        if not has_rail:
            if target.rail_position_si is not None:
                raise ValueError("单机械臂计划不得包含导轨目标")
            return None
        if target.rail_position_si is not None:
            return target.rail_position_si
        if current_rail_position_si is None:
            raise ValueError("省略 rail 时必须先冻结可信当前导轨位置")
        return current_rail_position_si


def _arm_phase(
    phase_id: str,
    target_ref: str,
    profile_ref: str,
    rail_position_si: float | None,
    payload_state: str,
) -> WorkCellPlanPhase:
    """构造一条机械臂阶段并携带全块固定导轨位置。"""

    return WorkCellPlanPhase(
        phase_id,
        WorkCellPhaseKind.ARM_MOVE,
        target_ref,
        profile_ref,
        rail_position_si,
        payload_state,
    )


__all__ = [
    "AccessMotionPlanCompiler",
    "ResolvedWorkCellPlan",
    "WorkCellPhaseKind",
    "WorkCellPlanPhase",
]
