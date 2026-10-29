"""公共时间参数化：把路点序列变成严格单调的梯形速度轨迹。

本模块不依赖 MoveIt、ROS 或 NumPy。输入是 ``PlanResult`` 或
``PlanWaypoint`` 序列，外加关节与导轨的速度/加速度限速；输出是同一几何路径
上的加密路点。每个路点段采用 rest-to-rest 梯形速度曲线（段间在顶点瞬时
静止）——比 MoveIt TOTG 保守，但结果是确定性的、可离线验证的：

* ``time_s`` 从 0 开始严格递增，多段运动的总时长大于 0；
* 每个关节/导轨的瞬时速度与加速度都不超过给定限速，按 ``sample_dt_s`` 加密
  后离散差分也满足限速；
* 单点或零位移输入折叠成一个 ``time_s=0`` 的路点，不伪造运动；
* 失败结果原样返回；成功结果才重新定时，``status``/``planner``/碰撞计数保持
  不变，只有 ``waypoints`` 与 ``diagnostics["time_parameterization"]`` 变化。

调用方仍须在参数化后重新做碰撞校验；本模块只处理时间，不改变几何路径。
"""

from __future__ import annotations

import math
import numbers
from collections.abc import Sequence
from typing import Any

from unilab_robot_contracts.planning import (
    PlanResult,
    PlanWaypoint,
    PlannerIdentity,
)

__all__ = ["PARAMETERIZER_IDENTITY", "parameterize"]

#: 裸路点输入时使用的诚实身份：重定时器不是规划器，不冒充 MoveIt。
PARAMETERIZER_IDENTITY = PlannerIdentity(
    name="unilab.trajectory_time",
    implementation="trapezoidal_rest_to_rest",
    version="v1",
)

_MOTION_EPS = 1e-12
_TIME_EPS = 1e-12


def _positive_finite(value: object, name: str) -> float:
    if isinstance(value, bool) or isinstance(value, (str, bytes)):
        raise ValueError(f"{name} 必须是正有限数")
    try:
        normalized = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} 必须是正有限数") from error
    if not math.isfinite(normalized) or normalized <= 0.0:
        raise ValueError(f"{name} 必须是正有限数")
    return normalized


def _axis_limits(
    value: object, dof: int, name: str
) -> tuple[float, ...]:
    """接受标量或逐轴序列，统一成 DOF 长的正有限数元组。"""

    if isinstance(value, numbers.Real) and not isinstance(value, bool):
        return (_positive_finite(value, name),) * dof
    if isinstance(value, (str, bytes)):
        raise TypeError(f"{name} 必须是正有限数或逐轴序列")
    try:
        items = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise TypeError(f"{name} 必须是正有限数或逐轴序列") from error
    if len(items) != dof:
        raise ValueError(f"{name} 长度必须等于关节维度 {dof}")
    return tuple(
        _positive_finite(item, f"{name}[{index}]")
        for index, item in enumerate(items)
    )


def _minimum_duration(distance: float, velocity: float, acceleration: float) -> float:
    """rest-to-rest 单轴最短时间：能到巡航速度用梯形，否则三角。"""

    if distance <= 0.0:
        return 0.0
    if distance >= velocity * velocity / acceleration:
        return distance / velocity + velocity / acceleration
    return 2.0 * math.sqrt(distance / acceleration)


def _profile(
    distance: float, velocity: float, acceleration: float, duration: float
) -> tuple[float, float, float, float]:
    """给定总时长反解单轴梯形曲线，返回 (v_peak, t_acc, t_cruise, a_used)。"""

    if distance <= 0.0:
        return 0.0, 0.0, duration, acceleration
    discriminant = max(
        0.0,
        acceleration * acceleration * duration * duration
        - 4.0 * acceleration * distance,
    )
    v_peak = 0.5 * (acceleration * duration - math.sqrt(discriminant))
    if v_peak <= 0.0:  # 数值退化时退回对称三角，仍满足时长与限速
        v_peak = 2.0 * distance / duration
        return v_peak, duration / 2.0, 0.0, 4.0 * distance / (duration * duration)
    v_peak = min(v_peak, velocity)
    t_acc = v_peak / acceleration
    t_cruise = max(0.0, duration - 2.0 * t_acc)
    return v_peak, t_acc, t_cruise, acceleration


def _distance_at(
    time_s: float,
    distance: float,
    v_peak: float,
    t_acc: float,
    t_cruise: float,
    acceleration: float,
    duration: float,
) -> float:
    """采样时刻的已走距离；端点严格夹到 0 与 distance。"""

    if time_s <= 0.0:
        return 0.0
    if time_s >= duration:
        return distance
    if time_s <= t_acc:
        covered = 0.5 * acceleration * time_s * time_s
    elif time_s <= t_acc + t_cruise:
        covered = 0.5 * acceleration * t_acc * t_acc + v_peak * (time_s - t_acc)
    else:
        remaining = duration - time_s
        covered = distance - 0.5 * acceleration * remaining * remaining
    return min(max(covered, 0.0), distance)


def _sample_times(
    profiles: Sequence[tuple[float, float, float, float]],
    duration: float,
    sample_dt_s: float,
) -> list[float]:
    """均匀采样 + 梯形拐点；去重后严格递增并以 duration 收尾。"""

    times = {0.0, duration}
    for _, t_acc, t_cruise, _ in profiles:
        if 0.0 < t_acc < duration:
            times.add(t_acc)
        if 0.0 < t_acc + t_cruise < duration:
            times.add(t_acc + t_cruise)
    step = 1
    while step * sample_dt_s < duration - _TIME_EPS:
        times.add(step * sample_dt_s)
        step += 1
    ordered = sorted(times)
    cleaned = [ordered[0]]
    for value in ordered[1:]:
        if value - cleaned[-1] > _TIME_EPS:
            cleaned.append(value)
    if cleaned[-1] < duration:
        cleaned[-1] = duration
    return cleaned


def _deduplicate(waypoints: Sequence[PlanWaypoint]) -> list[PlanWaypoint]:
    unique = [waypoints[0]]
    for waypoint in waypoints[1:]:
        previous = unique[-1]
        same_q = tuple(previous.q) == tuple(waypoint.q)
        same_rail = abs(previous.rail_m - waypoint.rail_m) <= _MOTION_EPS
        if not (same_q and same_rail):
            unique.append(waypoint)
    return unique


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
    duration: float,
    zero_motion: bool,
    sample_dt_s: float,
    rail_used: bool,
) -> PlanResult:
    info: dict[str, Any] = {
        "engine": "trapezoidal_rest_to_rest",
        "version": "v1",
        "sample_dt_s": sample_dt_s,
        "input_waypoints": input_count,
        "output_waypoints": len(waypoints),
        "duration_s": duration,
        "zero_motion": zero_motion,
        "rail_used": rail_used,
        "segment_stops": True,
    }
    if plan is not None:
        diagnostics = dict(plan.diagnostics)
        diagnostics["time_parameterization"] = info
        return PlanResult(
            status=plan.status,
            waypoints=tuple(waypoints),
            planner=plan.planner,
            collision_checks=plan.collision_checks,
            safe_pose_used=plan.safe_pose_used,
            diagnostics=diagnostics,
        )
    return PlanResult(
        status="success",
        waypoints=tuple(waypoints),
        planner=PARAMETERIZER_IDENTITY,
        diagnostics={"time_parameterization": info},
    )


def parameterize(
    plan: PlanResult | Sequence[PlanWaypoint],
    *,
    joint_velocity_limits_rad_s: Sequence[float] | float,
    joint_acceleration_limits_rad_s2: Sequence[float] | float,
    rail_velocity_limit_m_s: float | None = None,
    rail_acceleration_limit_m_s2: float | None = None,
    sample_dt_s: float = 0.02,
) -> PlanResult:
    """把 ``plan`` 的几何路径重新定时成严格单调的梯形速度轨迹。

    参数：``plan`` 是成功 ``PlanResult`` 或裸 ``PlanWaypoint`` 序列；
    ``joint_velocity_limits_rad_s``/``joint_acceleration_limits_rad_s2`` 可以是
    标量（所有轴同限速）或与 ``q`` 等长的逐轴序列；导轨只有在轨迹改变
    ``rail_m`` 时才必须给出速度/加速度限速。``sample_dt_s`` 是加密步长。

    返回：新的 ``PlanResult``。失败或空轨迹原样返回；裸路点输入得到
    ``status="success"`` 且身份为 ``PARAMETERIZER_IDENTITY``。
    异常：类型、维度、非有限/非正限速或导轨限速缺失时显式失败。
    """

    dt = _positive_finite(sample_dt_s, "sample_dt_s")
    result_plan, waypoints = _as_waypoints(plan)
    if result_plan is not None and result_plan.status != "success":
        # 失败结果可能带部分路点，不能因为重定时被当成可执行轨迹。
        return result_plan
    if not waypoints:
        if result_plan is not None:
            return result_plan
        raise ValueError("waypoints 不得为空")

    dof = len(waypoints[0].q)
    if any(len(item.q) != dof for item in waypoints):
        raise ValueError("所有路点必须具有相同关节维度")
    velocity_limits = _axis_limits(
        joint_velocity_limits_rad_s, dof, "joint_velocity_limits_rad_s"
    )
    acceleration_limits = _axis_limits(
        joint_acceleration_limits_rad_s2, dof, "joint_acceleration_limits_rad_s2"
    )
    if (rail_velocity_limit_m_s is None) != (rail_acceleration_limit_m_s2 is None):
        raise ValueError("rail 速度与加速度限速必须同时提供或同时省略")
    rail_velocity = (
        None
        if rail_velocity_limit_m_s is None
        else _positive_finite(rail_velocity_limit_m_s, "rail_velocity_limit_m_s")
    )
    rail_acceleration = (
        None
        if rail_acceleration_limit_m_s2 is None
        else _positive_finite(
            rail_acceleration_limit_m_s2, "rail_acceleration_limit_m_s2"
        )
    )

    unique = _deduplicate(waypoints)
    rail_used = any(
        abs(later.rail_m - earlier.rail_m) > _MOTION_EPS
        for earlier, later in zip(unique, unique[1:])
    )
    if rail_used and (rail_velocity is None or rail_acceleration is None):
        raise ValueError(
            "轨迹改变导轨时必须提供 rail_velocity_limit_m_s 与 "
            "rail_acceleration_limit_m_s2"
        )

    if len(unique) == 1:
        first = unique[0]
        return _build_result(
            result_plan,
            (PlanWaypoint(first.q, first.rail_m, 0.0, first.source),),
            input_count=len(waypoints),
            duration=0.0,
            zero_motion=True,
            sample_dt_s=dt,
            rail_used=False,
        )

    output: list[PlanWaypoint] = []
    offset = 0.0
    for segment_index, (start, end) in enumerate(zip(unique, unique[1:])):
        axes: list[tuple[float, float, float, float]] = []
        for index in range(dof):
            delta = end.q[index] - start.q[index]
            axes.append(
                (
                    abs(delta),
                    1.0 if delta >= 0.0 else -1.0,
                    velocity_limits[index],
                    acceleration_limits[index],
                )
            )
        rail_delta = end.rail_m - start.rail_m
        if abs(rail_delta) > _MOTION_EPS:
            assert rail_velocity is not None and rail_acceleration is not None
            axes.append(
                (
                    abs(rail_delta),
                    1.0 if rail_delta >= 0.0 else -1.0,
                    rail_velocity,
                    rail_acceleration,
                )
            )
        duration = max(
            _minimum_duration(distance, velocity, acceleration)
            for distance, _, velocity, acceleration in axes
        )
        if duration <= 0.0:  # 去重后不应出现；保持严格递增而不是制造重复时间
            continue
        profiles = [
            _profile(distance, velocity, acceleration, duration)
            for distance, _, velocity, acceleration in axes
        ]
        sample_times = _sample_times(profiles, duration, dt)
        for sample_index, sample_time in enumerate(sample_times):
            if segment_index > 0 and sample_index == 0:
                continue  # 上一段的末点已经占用了这个顶点时间
            q_values: list[float] = []
            for index in range(dof):
                distance, direction, _, _ = axes[index]
                v_peak, t_acc, t_cruise, a_used = profiles[index]
                q_values.append(
                    start.q[index]
                    + direction
                    * _distance_at(
                        sample_time,
                        distance,
                        v_peak,
                        t_acc,
                        t_cruise,
                        a_used,
                        duration,
                    )
                )
            rail_value = start.rail_m
            if len(axes) > dof:
                distance, direction, _, _ = axes[dof]
                v_peak, t_acc, t_cruise, a_used = profiles[dof]
                rail_value = start.rail_m + direction * _distance_at(
                    sample_time,
                    distance,
                    v_peak,
                    t_acc,
                    t_cruise,
                    a_used,
                    duration,
                )
            source = end.source if sample_time >= duration else start.source
            output.append(
                PlanWaypoint(tuple(q_values), rail_value, offset + sample_time, source)
            )
        offset += duration

    if not output:  # pragma: no cover - 去重后不可能为空，保留显式兜底
        first = unique[0]
        output.append(PlanWaypoint(first.q, first.rail_m, 0.0, first.source))
    return _build_result(
        result_plan,
        output,
        input_count=len(waypoints),
        duration=offset,
        zero_motion=False,
        sample_dt_s=dt,
        rail_used=rail_used,
    )
