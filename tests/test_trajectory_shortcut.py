"""公共轨迹 shortcut（不依赖 MoveIt）的红测试。

``shortcut`` 只做几何简化：用注入的碰撞回调判断两点之间的直线段是否可行，
删掉可被直连替代的中间路点。它必须保证：

* 输出点数下降，首尾路点（q/rail/time/source）不变；
* 输出里每一条相邻段都通过调用方注入的碰撞校验，绝不用捷径绕过障碍；
* 路点内容变化必须反映到 ``PlanResult.digest``；
* 简化不重新定时，调用方需再跑 ``parameterize`` 才能执行。
"""

from __future__ import annotations

import math

import pytest

from unilab_robot_contracts.planning import (
    PlanResult,
    PlanWaypoint,
    PlannerIdentity,
)
from unilab_robot_runtime.trajectory_shortcut import shortcut

IDENTITY = PlannerIdentity("fake-planner", "test", "1.0", {"engine": "unit-test"})


def _waypoint(x: float, y: float, time_s: float) -> PlanWaypoint:
    return PlanWaypoint((x, y), 0.0, time_s, "planner")


def _plan(points: list[PlanWaypoint], *, status: str = "success") -> PlanResult:
    return PlanResult(
        status=status,  # type: ignore[arg-type]
        waypoints=tuple(points),
        planner=IDENTITY,
        collision_checks=5,
        diagnostics={"source": "test"},
    )


def _point_segment_distance(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _circle_blocked(
    start: PlanWaypoint,
    end: PlanWaypoint,
    *,
    center: tuple[float, float] = (1.0, 1.0),
    radius: float = 0.3,
) -> bool:
    return (
        _point_segment_distance(center, start.q, end.q) <= radius  # type: ignore[arg-type]
    )


def test_shortcut_reduces_points_preserves_endpoints_and_changes_digest() -> None:
    points = [
        _waypoint(0.0, 0.0, 0.0),
        _waypoint(0.2, 0.1, 1.0),
        _waypoint(0.4, -0.1, 2.0),
        _waypoint(0.6, 0.1, 3.0),
        _waypoint(0.8, -0.1, 4.0),
        _waypoint(1.0, 0.0, 5.0),
    ]
    plan = _plan(points)
    calls: list[tuple[PlanWaypoint, PlanWaypoint]] = []

    def free(start: PlanWaypoint, end: PlanWaypoint) -> bool:
        calls.append((start, end))
        return True

    result = shortcut(plan, is_collision_free=free)
    assert len(result.waypoints) < len(plan.waypoints)
    assert result.waypoints[0] == plan.waypoints[0]
    assert result.waypoints[-1] == plan.waypoints[-1]
    assert result.digest != plan.digest
    assert result.status == "success"
    assert result.planner is plan.planner
    assert result.collision_checks == plan.collision_checks
    assert result.diagnostics["shortcut"]["removed"] > 0
    assert result.diagnostics["shortcut"]["output_waypoints"] == len(
        result.waypoints
    )
    assert calls, "碰撞回调必须被实际调用"


def test_shortcut_never_creates_a_segment_that_skips_the_obstacle() -> None:
    points = [
        _waypoint(0.0, 0.0, 0.0),
        _waypoint(1.0, 0.0, 1.0),
        _waypoint(2.0, 0.0, 2.0),
        _waypoint(2.0, 2.0, 3.0),
        _waypoint(0.0, 2.0, 4.0),
    ]
    plan = _plan(points)
    checks: list[tuple[PlanWaypoint, PlanWaypoint]] = []

    def is_collision_free(start: PlanWaypoint, end: PlanWaypoint) -> bool:
        checks.append((start, end))
        return not _circle_blocked(start, end)

    result = shortcut(plan, is_collision_free=is_collision_free)
    assert len(result.waypoints) < len(points)
    assert result.waypoints[0] == points[0]
    assert result.waypoints[-1] == points[-1]
    for start, end in zip(result.waypoints, result.waypoints[1:]):
        assert is_collision_free(start, end), (
            f"shortcut 输出了穿障碍的段: {start.q} -> {end.q}"
        )
    assert checks


def test_shortcut_returns_input_unchanged_when_no_segment_can_be_removed() -> None:
    points = [
        _waypoint(0.0, 0.0, 0.0),
        _waypoint(0.5, 0.5, 1.0),
        _waypoint(1.0, 0.0, 2.0),
    ]
    plan = _plan(points)
    adjacent = {
        (points[index], points[index + 1]) for index in range(len(points) - 1)
    }

    def adjacent_only(start: PlanWaypoint, end: PlanWaypoint) -> bool:
        return (start, end) in adjacent

    result = shortcut(plan, is_collision_free=adjacent_only)
    assert result.waypoints == plan.waypoints
    assert result.digest == plan.digest


def test_shortcut_rejects_an_input_path_that_already_fails_the_callback() -> None:
    plan = _plan(
        [
            _waypoint(0.0, 0.0, 0.0),
            _waypoint(1.0, 0.0, 1.0),
            _waypoint(2.0, 0.0, 2.0),
        ]
    )
    with pytest.raises(ValueError, match="碰撞|校验"):
        shortcut(plan, is_collision_free=lambda start, end: False)


def test_shortcut_requires_a_callable_and_validates_max_passes() -> None:
    plan = _plan([_waypoint(0.0, 0.0, 0.0), _waypoint(1.0, 0.0, 1.0)])
    with pytest.raises(TypeError):
        shortcut(plan, is_collision_free=None)  # type: ignore[arg-type]
    with pytest.raises(ValueError):
        shortcut(plan, is_collision_free=lambda start, end: True, max_passes=0)


def test_shortcut_passes_through_unsuccessful_or_short_plans_without_calling_back() -> None:
    calls: list[object] = []

    def record(*args: object) -> bool:
        calls.append(args)
        return True

    failed = _plan([], status="collision")
    assert shortcut(failed, is_collision_free=record) is failed
    two = _plan([_waypoint(0.0, 0.0, 0.0), _waypoint(1.0, 0.0, 1.0)])
    assert shortcut(two, is_collision_free=record) is two
    assert calls == []


def test_shortcut_accepts_bare_waypoints_with_an_honest_identity() -> None:
    points = [
        _waypoint(0.0, 0.0, 0.0),
        _waypoint(0.5, 0.0, 1.0),
        _waypoint(1.0, 0.0, 2.0),
    ]
    result = shortcut(points, is_collision_free=lambda start, end: True)
    assert result.status == "success"
    assert result.planner.name == "unilab.trajectory_shortcut"
    assert len(result.waypoints) < len(points)
    with pytest.raises(ValueError):
        shortcut([], is_collision_free=lambda start, end: True)


def test_shortcut_is_deterministic_and_propagates_callback_errors() -> None:
    points = [
        _waypoint(0.0, 0.0, 0.0),
        _waypoint(0.5, 0.0, 1.0),
        _waypoint(1.0, 0.0, 2.0),
        _waypoint(1.5, 0.0, 3.0),
    ]
    first = shortcut(points, is_collision_free=lambda start, end: True)
    second = shortcut(points, is_collision_free=lambda start, end: True)
    assert first.digest == second.digest

    def explode(start: PlanWaypoint, end: PlanWaypoint) -> bool:
        raise RuntimeError("collision service down")

    with pytest.raises(RuntimeError, match="collision service down"):
        shortcut(_plan(points), is_collision_free=explode)
