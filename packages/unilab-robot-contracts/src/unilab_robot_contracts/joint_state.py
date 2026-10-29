"""机械臂型号包与 OS 共用的关节反馈命名合同。"""

from __future__ import annotations

import math
import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

_DEVICE_ID = re.compile(r"^[A-Za-z0-9_]+$")


@dataclass(frozen=True, slots=True)
class JointStateNameMap:
    """把厂商真实反馈名精确映射为 Graph Device 限定名。

    该对象只做观测命名和数值校验，不是控制端口。每个 exact
    Arm 型号包必须显式构造它；OS 不允许前缀猜测或单设备回退。
    """

    device_id: str
    canonical_joint_names: tuple[str, ...]
    raw_to_canonical: Mapping[str, str]

    def __post_init__(self) -> None:
        """校验 Device 命名空间和一对一的厂商映射。"""

        if not _DEVICE_ID.fullmatch(self.device_id):
            raise ValueError("device_id 只能包含英文、数字和下划线")
        if not self.canonical_joint_names or any(
            not value.strip() for value in self.canonical_joint_names
        ):
            raise ValueError("canonical_joint_names 不得为空")
        if len(set(self.canonical_joint_names)) != len(self.canonical_joint_names):
            raise ValueError("canonical_joint_names 不得重复")
        if set(self.raw_to_canonical.values()) != set(self.canonical_joint_names):
            raise ValueError("厂商映射必须完整覆盖 exact Arm 关节")
        if len(set(self.raw_to_canonical)) != len(self.raw_to_canonical):
            raise ValueError("厂商关节名不得重复")

    @property
    def qualified_joint_names(self) -> tuple[str, ...]:
        """返回与型号顺序一致的完全限定关节名。"""

        return tuple(
            f"{self.device_id}_{name}" for name in self.canonical_joint_names
        )

    def qualify(
        self,
        raw_names: Sequence[str],
        positions: Sequence[float],
    ) -> dict[str, float]:
        """校验一帧真实反馈并返回完全限定的关节值。"""

        if len(raw_names) != len(positions):
            raise ValueError("JointState name/position 长度不一致")
        if len(set(raw_names)) != len(raw_names):
            raise ValueError("JointState 包含重复关节名")
        received = set(raw_names)
        expected = set(self.raw_to_canonical)
        if received != expected:
            missing = sorted(expected - received)
            unknown = sorted(received - expected)
            raise ValueError(
                f"JointState 与 exact Arm 映射不一致: missing={missing}, unknown={unknown}"
            )
        result: dict[str, float] = {}
        for raw_name, raw_position in zip(raw_names, positions, strict=True):
            position = float(raw_position)
            if not math.isfinite(position):
                raise ValueError(f"JointState 关节值必须为有限数: {raw_name}")
            canonical = self.raw_to_canonical[raw_name]
            result[f"{self.device_id}_{canonical}"] = position
        return result


__all__ = ["JointStateNameMap"]
