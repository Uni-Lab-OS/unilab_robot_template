"""MoveIt 单次动作的规划预算与重试旋钮（领域仓可共用）。

``plan_retry_attempts`` 由 OS ``MoveGroupActionRetry`` 读取 ``MoveItConfig``；
本模块对外暴露同一合同，并把 ``num_planning_attempts`` /
``allowed_planning_time`` 写进已创建的 MoveIt 客户端。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Type

DEFAULT_PLAN_RETRY_ATTEMPTS = 10
DEFAULT_NUM_PLANNING_ATTEMPTS = 3
DEFAULT_ALLOWED_PLANNING_TIME_S = 3.0


@dataclass(frozen=True, slots=True)
class MoveItPlanningBudget:
    """MoveGroup 派发前的规划预算；不是工作流（Workflow）执行重试。"""

    plan_retry_attempts: int = DEFAULT_PLAN_RETRY_ATTEMPTS
    num_planning_attempts: int = DEFAULT_NUM_PLANNING_ATTEMPTS
    allowed_planning_time_s: float = DEFAULT_ALLOWED_PLANNING_TIME_S


def _load_moveit_config() -> Type[Any] | None:
    """软导入 OS MoveItConfig；合同包不硬依赖 Uni-Lab-OS。"""

    try:
        from unilabos.config.config import MoveItConfig
    except ImportError:
        return None
    return MoveItConfig


def resolve_moveit_planning_budget() -> MoveItPlanningBudget:
    """解析当前进程生效的规划预算。

    优先读 ``unilabos.config.MoveItConfig``（含 local_config / 环境变量覆盖）；
    无 OS 时回落到模板默认值。
    """

    config = _load_moveit_config()
    if config is None:
        return MoveItPlanningBudget()
    try:
        return MoveItPlanningBudget(
            plan_retry_attempts=max(0, int(getattr(config, "plan_retry_attempts"))),
            num_planning_attempts=int(getattr(config, "num_planning_attempts")),
            allowed_planning_time_s=float(getattr(config, "allowed_planning_time")),
        )
    except (TypeError, ValueError, AttributeError):
        return MoveItPlanningBudget()


def apply_moveit_planning_budget(
    client: Any,
    *,
    budget: MoveItPlanningBudget | None = None,
) -> Any:
    """把规划预算写入 MoveIt 客户端字段并返回同一实例。

    参数：``client`` 需支持 ``num_planning_attempts`` 与 ``allowed_planning_time``；
    ``budget`` 缺省时调用 ``resolve_moveit_planning_budget()``。
    返回：同一 ``client``。异常：越界预算抛 ``ValueError``。
    安全：不重放已派发的动作（Action）；``plan_retry_attempts`` 仅作合同暴露，
    由 OS 动作重试层消费。
    """

    resolved = budget if budget is not None else resolve_moveit_planning_budget()
    attempts = int(resolved.num_planning_attempts)
    allowed = float(resolved.allowed_planning_time_s)
    if not 1 <= attempts <= 100:
        raise ValueError("MoveIt num_planning_attempts 必须位于 [1, 100]")
    if not 0.1 <= allowed <= 60.0:
        raise ValueError("MoveIt allowed_planning_time 必须位于 [0.1, 60]")
    client.num_planning_attempts = attempts
    client.allowed_planning_time = allowed
    return client


__all__ = [
    "DEFAULT_ALLOWED_PLANNING_TIME_S",
    "DEFAULT_NUM_PLANNING_ATTEMPTS",
    "DEFAULT_PLAN_RETRY_ATTEMPTS",
    "MoveItPlanningBudget",
    "apply_moveit_planning_budget",
    "resolve_moveit_planning_budget",
]
