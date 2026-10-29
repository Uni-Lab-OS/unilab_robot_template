"""公共轨迹 shortcut：用注入的碰撞回调删掉可被直连替代的路点。

本模块不依赖 MoveIt、ROS 或 NumPy；碰撞知识完全由调用方注入的
``is_collision_free(start, end)`` 提供。回调必须判断两点之间**整条直线段**
（关节空间与导轨坐标一起插值）是否无碰撞，而不只是两个端点。

规则：

* 先校验输入轨迹的每条相邻段都能通过回调；输入本身不合法时显式
  ``ValueError``，绝不把问题留给下游执行器；
* 只接受能通过回调的直连；输出轨迹的每条相邻段都会再校验一次；
* 首尾路点（``q``/``rail_m``/``time_s``/``source``）保持不变，点数和顺序只减
  不增，``PlanResult.digest`` 随内容变化；
* 不重新定时。删点后保留原时间戳（仍非递减），调用方必须再用
  ``trajectory_time.parameterize`` 按限速重定时，才能交给执行器。

无 ROS/MoveIt 环境可导入和测试。
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Any

from unilab_robot_contracts.planning import (
    PlanResult,
    PlanWaypoint,
    PlannerIdentity,
)

__all__ = ["SHORTCUT_IDENTITY", "shortcut"]

#: 裸路点输入时使用的诚实身份：shortcut 不是规划器，不冒充 MoveIt。
SHORTCUT_IDENTITY = PlannerIdentity(
    name="unilab.trajectory_shortcut",
    implementation="greedy_segment_shortcut",
    version="v1",
)

CollisionCheck = Callable[[PlanWaypoint, PlanWaypoint], bool]


def _as_waypoints(plan: object) -> tuple[PlanResult | None, tuple[PlanWaypoint, ...]]:
    if isinstance(plan, PlanResult):
        return plan, tuple(plan.waypoints)
    if isinstance(plan, (str, bytes)):
        raise TypeError("plan 必须是 PlanResult 或 PlanWaypoint 序列")
    try:
        items = tuple(plan)  # type: ignore[arg-type]
    except TypeError as error:
        raise TypeError("plan 必须是 PlanResult 或 PlanWaypoint 序列") from error
    if any(not isinstance(item, PlanWaypoint) for item in items):
        raise TypeError("plan 只能包含 PlanWaypoint")
    if not items:
        raise ValueError("waypoints 不得为空")
    return None, items


def _build_result(
    plan: PlanResult | None,
    waypoints: Sequence[PlanWaypoint],
    *,
    input_count: int,
    checks: int,
) -> PlanResult:
    info: dict[str, Any] = {
        "engine": "greedy_segment_shortcut",
        "version": "v1",
        "input_waypoints": input_count,
        "output_waypoints": len(waypoints),
        "removed": input_count - len(waypoints),
        "checks": checks,
        "reparameterized": False,
        "requires_reparameterization": True,
    }
    if plan is None:
        return PlanResult(
            status="success",
            waypoints=tuple(waypoints),
            planner=SHORTCUT_IDENTITY,
            diagnostics={"shortcut": info},
        )
    diagnostics = dict(plan.diagnostics)
    diagnostics["shortcut"] = info
    return PlanResult(
        status=plan.status,
        waypoints=tuple(waypoints),
        planner=plan.planner,
        collision_checks=plan.collision_checks,
        safe_pose_used=plan.safe_pose_used,
        diagnostics=diagnostics,
    )


def shortcut(
    plan: PlanResult | Sequence[PlanWaypoint],
    *,
    is_collision_free: CollisionCheck,
    max_passes: int = 4,
) -> PlanResult:
    """贪心删点，返回点更少但每段都通过碰撞回调的 ``PlanResult``。

    参数：``plan`` 是成功 ``PlanResult`` 或裸 ``PlanWaypoint`` 序列；
    ``is_collision_free(start, end)`` 由调用方注入，必须检查整条直线段；
    ``max_passes`` 是最多删点轮数（每轮从每个起点尝试最远的可行直连）。

    返回：新的 ``PlanResult``；失败结果、少于 3 个路点或没有可删点时原样
    返回（成功裸序列会包装成 ``SHORTCUT_IDENTITY`` 的结果）。
    异常：回调不可调用、输入轨迹不通过回调（``ValueError``）或回调自身抛错。
    """

    if not callable(is_collision_free):
        raise TypeError("is_collision_free 必须是可调用对象")
    if isinstance(max_passes, bool) or not isinstance(max_passes, int) or max_passes < 1:
        raise ValueError("max_passes 必须是正整数")
    result_plan, waypoints = _as_waypoints(plan)
    if result_plan is not None and result_plan.status != "success":
        # 失败结果可能带部分路点，不能因为简化被当成可执行轨迹。
        return result_plan
    if len(waypoints) < 3:
        if result_plan is not None:
            return result_plan
        return _build_result(
            None, waypoints, input_count=len(waypoints), checks=0
        )

    checks = 0
    for start, end in zip(waypoints, waypoints[1:]):
        checks += 1
        if not is_collision_free(start, end):
            raise ValueError(
                "输入轨迹本身不能通过碰撞校验，拒绝 shortcut："
                f"{start.q} -> {end.q}"
            )

    points = list(waypoints)
    for _ in range(max_passes):
        changed = False
        index = 0
        while index < len(points) - 2:
            candidate = len(points) - 1
            while candidate > index + 1:
                checks += 1
                if is_collision_free(points[index], points[candidate]):
                    points = points[: index + 1] + points[candidate:]
                    changed = True
                    break
                candidate -= 1
            index += 1
        if not changed:
            break

    if len(points) == len(waypoints):
        if result_plan is not None:
            return result_plan
        return _build_result(
            None, waypoints, input_count=len(waypoints), checks=checks
        )

    # 不变量：输出的每条相邻段都必须通过回调；这里是第二道闸门。
    for start, end in zip(points, points[1:]):
        checks += 1
        if not is_collision_free(start, end):
            raise RuntimeError(
                "shortcut 产生了未通过碰撞校验的段，拒绝返回结果："
                f"{start.q} -> {end.q}"
            )

    return _build_result(
        result_plan,
        points,
        input_count=len(waypoints),
        checks=checks,
    )
