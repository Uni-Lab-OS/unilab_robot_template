"""机械臂引擎接缝的 fake scene 行为：写入/读回、轨迹顺序与可选能力。"""

from __future__ import annotations

import math
from typing import Any

import pytest
from unilab_robot_runtime.engines import ArmEnginePort, KinematicArmEngine


class _FakeScene:
    """记录 ``set_pose``/``update`` 的 duck-typed 场景替身，不依赖 bpy。"""

    def __init__(self) -> None:
        self.poses: dict[str, tuple[tuple[float, ...], ...]] = {}
        self.set_calls: list[tuple[str, tuple[tuple[float, ...], ...]]] = []
        self.update_count = 0

    def set_pose(self, name: str, matrix: Any) -> None:
        copied = tuple(tuple(float(value) for value in row) for row in matrix)
        self.set_calls.append((name, copied))
        self.poses[name] = copied

    def update(self) -> None:
        self.update_count += 1

    def pose(self, name: str) -> tuple[tuple[float, ...], ...]:
        return self.poses[name]


def _engine(scene: _FakeScene | None = None) -> KinematicArmEngine:
    """按两个显式关节映射构造引擎，默认使用记录型 fake scene。"""

    return KinematicArmEngine(
        scene if scene is not None else _FakeScene(),
        {"j1": "joint/1", "j2": "joint/2"},
    )


def test_engine_is_arm_engine_port() -> None:
    """Kinematic 引擎实现同一端口，设备包无需按后端分支。"""

    assert isinstance(_engine(), ArmEnginePort)
    assert not isinstance(object(), ArmEnginePort)


def test_apply_writes_poses_then_reads_back() -> None:
    """apply 按关节名顺序写入场景，update 后 read 返回同一数值。"""

    scene = _FakeScene()
    engine = _engine(scene)

    engine.apply_joint_positions(("j1", "j2"), (0.25, -0.5))

    assert [name for name, _ in scene.set_calls] == ["joint/1", "joint/2"]
    assert scene.update_count == 1
    assert engine.read_joint_positions() == {"j1": 0.25, "j2": -0.5}


def test_apply_rejects_unknown_joint_before_mutating_scene() -> None:
    """缺关节是输入错误，拒绝时不能留下部分写入。"""

    scene = _FakeScene()
    engine = _engine(scene)

    with pytest.raises(ValueError, match="unknown_joint"):
        engine.apply_joint_positions(("j1", "missing"), (0.1, 0.2))

    assert scene.set_calls == []
    assert scene.update_count == 0


@pytest.mark.parametrize(
    ("names", "positions", "match"),
    [
        (("j1",), (0.1, 0.2), "一一对应"),
        (("j1", "j1"), (0.1, 0.2), "重复"),
        (("j1", "j2"), (0.1, float("nan")), "有限数"),
        ((), (), "不能为空"),
    ],
)
def test_apply_rejects_invalid_joint_inputs(
    names: tuple[str, ...],
    positions: tuple[float, ...],
    match: str,
) -> None:
    """关节名/位置必须一一对应、唯一且有限。"""

    engine = _engine()

    with pytest.raises(ValueError, match=match):
        engine.apply_joint_positions(names, positions)


def test_trajectory_applies_each_point_in_order_without_new_clock() -> None:
    """默认轨迹实现逐点调用 apply，不插值、不睡眠、不新建时钟。"""

    scene = _FakeScene()
    engine = _engine(scene)

    engine.apply_trajectory(
        ("j1", "j2"),
        ((0.0, 0.0), (1.0, 2.0), (2.0, 4.0)),
        (0.0, 0.5, 1.0),
    )

    assert [name for name, _ in scene.set_calls] == [
        "joint/1",
        "joint/2",
        "joint/1",
        "joint/2",
        "joint/1",
        "joint/2",
    ]
    assert scene.update_count == 3
    assert engine.read_joint_positions() == {"j1": 2.0, "j2": 4.0}


@pytest.mark.parametrize(
    ("times", "match"),
    [
        ((0.0, 0.0, 1.0), "严格递增"),
        ((0.0, 1.0), "一一对应"),
        ((0.0, float("inf"), 1.0), "有限数"),
    ],
)
def test_trajectory_rejects_invalid_times(
    times: tuple[float, ...],
    match: str,
) -> None:
    """轨迹时间必须逐点对应、有限且严格递增。"""

    engine = _engine()

    with pytest.raises(ValueError, match=match):
        engine.apply_trajectory(
            ("j1", "j2"),
            ((0.0, 0.0), (1.0, 2.0), (2.0, 4.0)),
            times,
        )


def test_empty_trajectory_is_rejected() -> None:
    """空轨迹没有可应用的点，应显式拒绝而不是静默成功。"""

    engine = _engine()

    with pytest.raises(ValueError, match="至少"):
        engine.apply_trajectory(("j1", "j2"), (), ())


def test_attach_and_release_are_optional_and_default_unsupported() -> None:
    """附着是可选能力；默认引擎明确报告不支持。"""

    engine = _engine()

    with pytest.raises(NotImplementedError, match="unsupported"):
        engine.attach("beaker", "tcp")
    with pytest.raises(NotImplementedError, match="unsupported"):
        engine.release("beaker")


def test_engine_requires_duck_typed_scene_and_joint_map() -> None:
    """scene 必须提供 set_pose/update/pose，关节映射不得为空。"""

    with pytest.raises(TypeError, match="set_pose"):
        KinematicArmEngine(object(), {"j1": "joint/1"})
    with pytest.raises(ValueError, match="至少"):
        KinematicArmEngine(_FakeScene(), {})


def test_read_rejects_non_4x4_scene_pose() -> None:
    """读回只接受场景给出的 4x4 齐次矩阵，避免静默错读。"""

    class _BadScene(_FakeScene):
        def pose(self, name: str) -> Any:
            return ((1.0, 0.0), (0.0, 1.0))

    engine = _engine(_BadScene())

    with pytest.raises(ValueError, match="4x4"):
        engine.read_joint_positions()


def test_engine_reports_finite_values_from_scene() -> None:
    """场景返回非有限数值时读回失败，不能把 NaN 当到位。"""

    class _NaNScene(_FakeScene):
        def pose(self, name: str) -> Any:
            return (
                (1.0, 0.0, 0.0, math.nan),
                (0.0, 1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
            )

    engine = _engine(_NaNScene())

    with pytest.raises(ValueError, match="有限数"):
        engine.read_joint_positions()
