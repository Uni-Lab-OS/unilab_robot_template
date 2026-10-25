"""机械臂引擎端口：关节写入/读回、逐点轨迹与可选附着能力。"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Protocol, runtime_checkable


def normalize_joint_names(names: Sequence[str]) -> tuple[str, ...]:
    """把关节名规范为去空白、非空且不重复的有序 tuple。"""

    if isinstance(names, (str, bytes)) or not isinstance(names, Sequence):
        raise TypeError("joint_names 必须是关节名序列")
    normalized = tuple(str(name).strip() for name in names)
    if not normalized:
        raise ValueError("joint_names 不能为空")
    if any(not name for name in normalized):
        raise ValueError("joint_names 不能包含空关节名")
    if len(set(normalized)) != len(normalized):
        raise ValueError("joint_names 不得重复")
    return normalized


def normalize_positions(
    positions: Sequence[float],
    names: tuple[str, ...],
) -> tuple[float, ...]:
    """把关节位置规范为与关节名一一对应的有限浮点数。"""

    if isinstance(positions, (str, bytes)) or not isinstance(positions, Sequence):
        raise TypeError("positions 必须是数值序列")
    if len(positions) != len(names):
        raise ValueError("positions 必须与 joint_names 一一对应")
    values: list[float] = []
    for name, value in zip(names, positions, strict=True):
        if isinstance(value, bool):
            raise ValueError(f"关节 {name} 位置必须是有限数")
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError(f"关节 {name} 位置必须是有限数") from error
        if not math.isfinite(number):
            raise ValueError(f"关节 {name} 位置必须是有限数")
        values.append(number)
    return tuple(values)


def normalize_trajectory(
    joint_names: Sequence[str],
    points: Sequence[Sequence[float]],
    times: Sequence[float],
) -> tuple[tuple[str, ...], tuple[tuple[float, ...], ...], tuple[float, ...]]:
    """校验逐点轨迹；按点调用 apply，不插值、不推进时间。"""

    names = normalize_joint_names(joint_names)
    if (
        isinstance(points, (str, bytes))
        or not isinstance(points, Sequence)
        or not points
    ):
        raise ValueError("轨迹至少需要一个点")
    if isinstance(times, (str, bytes)) or not isinstance(times, Sequence):
        raise TypeError("times 必须是数值序列")
    if len(times) != len(points):
        raise ValueError("times 必须与轨迹点一一对应")
    stamps: list[float] = []
    for value in times:
        if isinstance(value, bool):
            raise ValueError("times 必须是有限数")
        try:
            stamp = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError("times 必须是有限数") from error
        if not math.isfinite(stamp):
            raise ValueError("times 必须是有限数")
        stamps.append(stamp)
    if any(later <= earlier for earlier, later in zip(stamps, stamps[1:])):
        raise ValueError("times 必须严格递增")
    normalized_points = tuple(
        normalize_positions(point, names) for point in points
    )
    return names, normalized_points, tuple(stamps)


@runtime_checkable
class ArmEnginePort(Protocol):
    """设备包可复用的机械臂引擎接缝；关节名是唯一身份，不绑定型号。"""

    def apply_joint_positions(
        self,
        joint_names: Sequence[str],
        positions: Sequence[float],
    ) -> None:
        """写入一组关节位置，并让场景求值到可读回状态。"""

    def read_joint_positions(self) -> dict[str, float]:
        """从已求值场景读回全部已映射关节位置。"""

    def apply_trajectory(
        self,
        joint_names: Sequence[str],
        points: Sequence[Sequence[float]],
        times: Sequence[float],
    ) -> None:
        """按给定时间戳逐点应用轨迹，不新建时钟。"""

    def attach(self, child: str, parent: str) -> None:
        """可选能力：把场景对象附着到父对象。"""

    def release(self, child: str) -> None:
        """可选能力：解除场景对象父级并保留世界姿态。"""


class ArmEngineBase:
    """``ArmEnginePort`` 的默认实现：逐点轨迹与明确不支持的可选附着。"""

    def apply_trajectory(
        self,
        joint_names: Sequence[str],
        points: Sequence[Sequence[float]],
        times: Sequence[float],
    ) -> None:
        """按段调用 ``apply_joint_positions``，不睡眠、不插值、不新建时钟。"""

        names, normalized_points, _ = normalize_trajectory(joint_names, points, times)
        for point in normalized_points:
            self.apply_joint_positions(names, point)

    def attach(self, child: str, parent: str) -> None:
        """默认引擎不拥有场景父子关系，明确拒绝而不是静默成功。"""

        raise NotImplementedError("arm_engine_attach_unsupported")

    def release(self, child: str) -> None:
        """默认引擎不拥有场景父子关系，明确拒绝而不是静默成功。"""

        raise NotImplementedError("arm_engine_release_unsupported")

    def apply_joint_positions(
        self,
        joint_names: Sequence[str],
        positions: Sequence[float],
    ) -> None:
        """具体引擎必须实现关节写入。"""

        raise NotImplementedError("arm_engine_apply_unsupported")

    def read_joint_positions(self) -> dict[str, float]:
        """具体引擎必须实现关节读回。"""

        raise NotImplementedError("arm_engine_read_unsupported")


__all__ = [
    "ArmEngineBase",
    "ArmEnginePort",
    "normalize_joint_names",
    "normalize_positions",
    "normalize_trajectory",
]
