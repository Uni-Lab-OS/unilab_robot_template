"""把关节名映射到场景对象的纯 Python 机械臂引擎；不依赖 bpy。"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

from .base import ArmEngineBase, normalize_joint_names, normalize_positions

_METHODS = ("set_pose", "update", "pose")


def _joint_pose(value: float) -> tuple[tuple[float, float, float, float], ...]:
    """把标量关节值编码为标记对象沿场景 X 轴的平移。

    本引擎只负责 OS ``Scene`` 的坐标标记读写，不声明铰链轴或 URDF 正解；
    真实型号语义（旋转轴、连杆链、TCP）仍由领域包在场景之上计算。
    """

    return (
        (1.0, 0.0, 0.0, value),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _joint_value(name: str, matrix: Any) -> float:
    """从 4x4 世界矩阵读回标记对象的 X 平移，并拒绝非有限结果。"""

    try:
        rows = tuple(tuple(row) for row in matrix)
    except TypeError as error:
        raise ValueError(f"关节 {name} 场景姿态必须是 4x4 齐次矩阵") from error
    if len(rows) != 4 or any(len(row) != 4 for row in rows):
        raise ValueError(f"关节 {name} 场景姿态必须是 4x4 齐次矩阵")
    try:
        value = float(rows[0][3])
    except (TypeError, ValueError) as error:
        raise ValueError(f"关节 {name} 位置必须是有限数") from error
    if not math.isfinite(value):
        raise ValueError(f"关节 {name} 位置必须是有限数")
    return value


class KinematicArmEngine(ArmEngineBase):
    """在注入的 duck-typed ``scene`` 上读写关节标记对象。

    参数：``scene`` 需提供 ``set_pose(name, matrix)``/``update()``/``pose(name)``；
    ``joint_objects`` 是关节名到场景对象名的显式映射。返回：无。
    异常：scene 缺少方法、映射为空或关节未知时拒绝。
    """

    def __init__(self, scene: Any, joint_objects: Mapping[str, str]) -> None:
        missing = [
            name for name in _METHODS if not callable(getattr(scene, name, None))
        ]
        if missing:
            raise TypeError(
                "KinematicArmEngine.scene 必须提供 set_pose/update/pose: "
                + ", ".join(missing)
            )
        if not isinstance(joint_objects, Mapping) or not joint_objects:
            raise ValueError("joint_objects 至少需要一个关节映射")
        normalized: dict[str, str] = {}
        for joint, target in joint_objects.items():
            name = str(joint).strip()
            object_name = str(target).strip()
            if not name or not object_name:
                raise ValueError("joint_objects 的关节名与对象名都不能为空")
            if name in normalized:
                raise ValueError(f"joint_objects 关节名不得重复: {name}")
            normalized[name] = object_name
        self._scene = scene
        self._joint_objects = normalized

    @property
    def scene(self) -> Any:
        """返回注入的场景；生命周期仍归装配方。"""

        return self._scene

    @property
    def joint_objects(self) -> dict[str, str]:
        """返回关节到对象名映射的副本。"""

        return dict(self._joint_objects)

    def apply_joint_positions(
        self,
        joint_names: Sequence[str],
        positions: Sequence[float],
    ) -> None:
        """先校验全部输入与映射，再按关节顺序写入并求值一次场景。"""

        names = normalize_joint_names(joint_names)
        values = normalize_positions(positions, names)
        targets: list[tuple[str, float]] = []
        for name, value in zip(names, values, strict=True):
            target = self._joint_objects.get(name)
            if target is None:
                raise ValueError(f"arm_engine_unknown_joint: {name}")
            targets.append((target, value))
        for target, value in targets:
            self._scene.set_pose(target, _joint_pose(value))
        self._scene.update()

    def read_joint_positions(self) -> dict[str, float]:
        """按映射顺序从已求值场景读回全部关节位置。"""

        return {
            name: _joint_value(name, self._scene.pose(target))
            for name, target in self._joint_objects.items()
        }


__all__ = ["KinematicArmEngine"]
