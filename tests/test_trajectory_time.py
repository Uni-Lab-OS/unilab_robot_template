"""公共时间参数化（不依赖 MoveIt）的红测试。

合同只要求路点时间非递减；执行侧需要严格单调、总时长大于零、并且每个关节和
导轨的速度/加速度都不超过显式限速。这里用梯形速度曲线（rest-to-rest）逐段
参数化，并按 ``sample_dt_s`` 加密，使离散差分也能验证限速。单点或零位移输入
没有可参数化的运动，输出单个 ``time_s=0`` 的路点。
"""

from __future__ import annotations

import math

import pytest

from unilab_robot_contracts.planning import (
    PlanResult,
    PlanWaypoint,
    PlannerIdentity,
)
from unilab_robot_runtime.trajectory_time import parameterize

IDENTITY = PlannerIdentity(
    "fake-planner", "test", "1.0", {"engine": "unit-test"}
)


def _waypoint(q: tuple[float, ...], rail: float, time_s: float) -> PlanWaypoint:
    return PlanWaypoint(q, rail, time_s, "planner")


def _plan(waypoints: list[PlanWaypoint], *, status: str = "success") -> PlanResult:
    return PlanResult(
        status=status,  # type: ignore[arg-type]
        waypoints=tuple(waypoints),
        planner=IDENTITY,
        collision_checks=7,
        diagnostics={"source": "test"},
    )


def _discrete_motion(
    points: tuple[PlanWaypoint, ...],
) -> tuple[list[float], list[list[float]], list[list[float]]]:
    """返回 dt、每步速度、每步加速度（相邻平均速度差 / 时间中点差）。"""

    dts = [
        later.time_s - earlier.time_s
        for earlier, later in zip(points, points[1:])
    ]
    velocities = [
        [
            (later.q[index] - earlier.q[index]) / dt
            for index in range(len(earlier.q))
        ]
        for (earlier, later), dt in zip(zip(points, points[1:]), dts)
    ]
    accelerations: list[list[float]] = []
    for index in range(1, len(velocities)):
        span = 0.5 * (dts[index - 1] + dts[index])
        accelerations.append(
            [
                (velocities[index][axis] - velocities[index - 1][axis]) / span
                for axis in range(len(velocities[index]))
            ]
        )
    return dts, velocities, accelerations


def test_parameterize_is_strictly_monotonic_with_positive_duration() -> None:
    plan = _plan(
        [
            _waypoint((0.0,), 0.0, 0.0),
            _waypoint((0.5,), 0.0, 0.1),
            _waypoint((1.0,), 0.0, 0.2),
        ]
    )
    result = parameterize(
        plan,
        joint_velocity_limits_rad_s=0.5,
        joint_acceleration_limits_rad_s2=1.0,
    )
    assert result.status == "success"
    assert result.planner is plan.planner
    assert result.collision_checks == 7
    times = [point.time_s for point in result.waypoints]
    assert times[0] == 0.0
    assert times[-1] > 0.0
    assert all(later > earlier for earlier, later in zip(times, times[1:]))
    assert result.waypoints[0].q == (0.0,)
    assert result.waypoints[-1].q == (1.0,)
    assert result.diagnostics["time_parameterization"]["engine"] == (
        "trapezoidal_rest_to_rest"
    )


def test_parameterize_respects_joint_velocity_and_acceleration_limits() -> None:
    plan = _plan(
        [
            _waypoint((0.0,), 0.0, 0.0),
            _waypoint((0.5,), 0.0, 0.1),
            _waypoint((1.0,), 0.0, 0.2),
        ]
    )
    velocity_limit = 0.5
    acceleration_limit = 1.0
    result = parameterize(
        plan,
        joint_velocity_limits_rad_s=velocity_limit,
        joint_acceleration_limits_rad_s2=acceleration_limit,
        sample_dt_s=0.02,
    )
    _, velocities, accelerations = _discrete_motion(result.waypoints)
    peak_velocity = max(abs(value) for step in velocities for value in step)
    peak_acceleration = max(abs(value) for step in accelerations for value in step)
    assert peak_velocity <= velocity_limit * (1.0 + 1e-9)
    assert peak_acceleration <= acceleration_limit * (1.0 + 1e-6)
    # 限速确实被用满（梯形巡航段），而不是用一个任意大的固定时长。
    assert peak_velocity >= velocity_limit * 0.99


def test_parameterize_respects_rail_velocity_and_acceleration_limits() -> None:
    plan = _plan(
        [
            _waypoint((0.0,), 0.0, 0.0),
            _waypoint((0.0,), 0.3, 0.1),
        ]
    )
    result = parameterize(
        plan,
        joint_velocity_limits_rad_s=1.0,
        joint_acceleration_limits_rad_s2=2.0,
        rail_velocity_limit_m_s=0.1,
        rail_acceleration_limit_m_s2=0.2,
        sample_dt_s=0.02,
    )
    rails = [point.rail_m for point in result.waypoints]
    assert rails[0] == 0.0 and rails[-1] == 0.3
    # d/v + v/a = 3.0 + 0.5 = 3.5 s，是这条一维轨迹的最短梯形时长。
    assert 3.49 <= result.waypoints[-1].time_s <= 3.51
    dts = [
        later.time_s - earlier.time_s
        for earlier, later in zip(result.waypoints, result.waypoints[1:])
    ]
    velocities = [
        (later.rail_m - earlier.rail_m) / dt
        for earlier, later, dt in zip(
            result.waypoints, result.waypoints[1:], dts
        )
    ]
    assert max(abs(value) for value in velocities) <= 0.1 * (1.0 + 1e-9)


def test_parameterize_rejects_rail_motion_without_rail_limits() -> None:
    plan = _plan(
        [
            _waypoint((0.0,), 0.0, 0.0),
            _waypoint((0.0,), 0.3, 0.1),
        ]
    )
    with pytest.raises(ValueError, match="rail"):
        parameterize(
            plan,
            joint_velocity_limits_rad_s=1.0,
            joint_acceleration_limits_rad_s2=2.0,
        )


def test_parameterize_handles_single_point_and_zero_displacement() -> None:
    single = _plan([_waypoint((0.4,), 0.2, 9.0)])
    result = parameterize(
        single,
        joint_velocity_limits_rad_s=1.0,
        joint_acceleration_limits_rad_s2=1.0,
    )
    assert len(result.waypoints) == 1
    assert result.waypoints[0].time_s == 0.0
    assert result.waypoints[0].q == (0.4,)
    assert result.waypoints[0].rail_m == 0.2
    assert result.diagnostics["time_parameterization"]["zero_motion"] is True

    duplicated = _plan(
        [
            _waypoint((0.4,), 0.2, 0.0),
            _waypoint((0.4,), 0.2, 1.0),
            _waypoint((0.4,), 0.2, 2.0),
        ]
    )
    collapsed = parameterize(
        duplicated,
        joint_velocity_limits_rad_s=1.0,
        joint_acceleration_limits_rad_s2=1.0,
    )
    assert len(collapsed.waypoints) == 1
    assert collapsed.waypoints[0].time_s == 0.0
    assert collapsed.waypoints[0].q == (0.4,)


def test_parameterize_passes_failure_results_through_and_rejects_empty_sequences() -> None:
    failed = _plan([], status="collision")
    assert parameterize(
        failed,
        joint_velocity_limits_rad_s=1.0,
        joint_acceleration_limits_rad_s2=1.0,
    ) is failed
    with pytest.raises(ValueError):
        parameterize(
            [],
            joint_velocity_limits_rad_s=1.0,
            joint_acceleration_limits_rad_s2=1.0,
        )


def test_parameterize_accepts_bare_waypoints_with_an_honest_identity() -> None:
    result = parameterize(
        [_waypoint((0.0,), 0.0, 0.0), _waypoint((0.2,), 0.0, 0.1)],
        joint_velocity_limits_rad_s=(0.5,),
        joint_acceleration_limits_rad_s2=(1.0,),
    )
    assert result.status == "success"
    assert result.planner.name == "unilab.trajectory_time"
    assert result.planner.implementation == "trapezoidal_rest_to_rest"
    assert result.waypoints[-1].q == (0.2,)


def test_parameterize_validates_limits_sample_step_and_input_types() -> None:
    plan = _plan(
        [_waypoint((0.0, 0.0), 0.0, 0.0), _waypoint((0.2, 0.2), 0.0, 0.1)]
    )
    kwargs = {
        "joint_velocity_limits_rad_s": 1.0,
        "joint_acceleration_limits_rad_s2": 1.0,
    }
    with pytest.raises(ValueError):
        parameterize(plan, **{**kwargs, "joint_velocity_limits_rad_s": (1.0,)})
    for bad in (0.0, -1.0, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            parameterize(plan, **{**kwargs, "joint_velocity_limits_rad_s": bad})
    with pytest.raises(ValueError):
        parameterize(plan, **{**kwargs, "sample_dt_s": 0.0})
    with pytest.raises(ValueError):
        parameterize(plan, **{**kwargs, "sample_dt_s": float("nan")})
    with pytest.raises(ValueError, match="rail"):
        parameterize(plan, **kwargs, rail_velocity_limit_m_s=0.1)
    with pytest.raises(TypeError):
        parameterize(42, **kwargs)  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        parameterize([(0.0, 0.0)], **kwargs)  # type: ignore[list-item]


def test_parameterize_is_deterministic() -> None:
    plan = _plan(
        [_waypoint((0.0,), 0.0, 0.0), _waypoint((0.5,), 0.0, 0.1)]
    )
    kwargs = {
        "joint_velocity_limits_rad_s": 0.5,
        "joint_acceleration_limits_rad_s2": 1.0,
    }
    first = parameterize(plan, **kwargs)
    second = parameterize(plan, **kwargs)
    assert first.digest == second.digest
    assert math.isclose(first.waypoints[-1].time_s, 1.5)
