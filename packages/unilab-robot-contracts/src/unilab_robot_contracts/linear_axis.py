"""三轴/导轨等有序轴链的合同：轴序、限位、单位、frame 与稳定摘要。"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

_AXIS_UNITS = frozenset({"m", "rad"})
_SCHEMA = "unilab.linear-axis-chain/v1"


@dataclass(frozen=True, slots=True)
class LinearAxis:
    """一条有序线性轴：名称、SI 单位、闭区间限位与所在 frame。"""

    name: str
    unit: str
    limits: tuple[float, float]
    frame: str

    def __post_init__(self) -> None:
        """校验轴名、单位、有限限位与 frame，并规范化存储。"""

        name = str(self.name).strip()
        if not name:
            raise ValueError("LinearAxis.name 不能为空")
        unit = str(self.unit).strip()
        if unit not in _AXIS_UNITS:
            raise ValueError(f"LinearAxis.unit 只允许 m/rad: {unit or '<empty>'}")
        frame = str(self.frame).strip()
        if not frame:
            raise ValueError("LinearAxis.frame 不能为空")
        if (
            isinstance(self.limits, (str, bytes))
            or not isinstance(self.limits, Sequence)
            or len(self.limits) != 2
        ):
            raise ValueError("LinearAxis.limits 必须是两个数值 (lower, upper)")
        raw_limits = tuple(self.limits)
        if any(isinstance(value, bool) for value in raw_limits):
            raise ValueError(f"LinearAxis {name} 限位必须是有限数")
        try:
            lower, upper = (float(value) for value in raw_limits)
        except (TypeError, ValueError) as error:
            raise ValueError("LinearAxis.limits 必须是两个数值 (lower, upper)") from error
        if not math.isfinite(lower) or not math.isfinite(upper):
            raise ValueError(f"LinearAxis {name} 限位必须是有限数")
        if lower > upper:
            raise ValueError(f"LinearAxis {name} 下限不得大于上限")
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "unit", unit)
        object.__setattr__(self, "limits", (lower, upper))
        object.__setattr__(self, "frame", frame)

    @property
    def lower(self) -> float:
        """返回闭区间下限。"""

        return self.limits[0]

    @property
    def upper(self) -> float:
        """返回闭区间上限。"""

        return self.limits[1]

    def as_mapping(self) -> dict[str, Any]:
        """返回摘要使用的显式字段，不含时间、路径或随机数。"""

        return {
            "name": self.name,
            "unit": self.unit,
            "limits": [self.lower, self.upper],
            "frame": self.frame,
        }


@dataclass(frozen=True, slots=True)
class LinearAxisChain:
    """一条有序轴链；轴序本身参与身份，摘要只由显式轴事实决定。"""

    axes: tuple[LinearAxis, ...]

    def __post_init__(self) -> None:
        """要求非空、有序且轴名唯一的 ``LinearAxis`` 序列。"""

        if isinstance(self.axes, (str, bytes)) or not isinstance(self.axes, Sequence):
            raise TypeError("LinearAxisChain.axes 必须是 LinearAxis 序列")
        axes = tuple(self.axes)
        if not axes:
            raise ValueError("LinearAxisChain 至少需要一个轴")
        if any(not isinstance(axis, LinearAxis) for axis in axes):
            raise TypeError("LinearAxisChain.axes 只能包含 LinearAxis")
        names = tuple(axis.name for axis in axes)
        if len(set(names)) != len(names):
            raise ValueError("LinearAxisChain 轴名不得重复")
        object.__setattr__(self, "axes", axes)

    @property
    def names(self) -> tuple[str, ...]:
        """按声明顺序返回轴名。"""

        return tuple(axis.name for axis in self.axes)

    @property
    def digest(self) -> str:
        """返回由轴序、限位、单位与 frame 稳定计算的 SHA-256。"""

        return linear_axis_chain_digest(self)

    def as_mapping(self) -> dict[str, Any]:
        """返回可写入配置/证据的显式链描述。"""

        return {
            "schema": _SCHEMA,
            "axes": [axis.as_mapping() for axis in self.axes],
        }


def linear_axis_chain_digest(axes: Sequence[LinearAxis]) -> str:
    """由显式轴事实稳定计算轴链摘要。

    参数：有序 ``LinearAxis`` 序列或 ``LinearAxisChain``。返回：64 位小写
    十六进制 SHA-256。异常：轴序非法、限位非有限或单位不受支持时拒绝。
    安全：摘要不接收时间、随机数、本机路径或未声明字段，因此同一显式
    输入在任何机器上复算一致；交换轴序会得到不同摘要。
    """

    chain = axes if isinstance(axes, LinearAxisChain) else LinearAxisChain(tuple(axes))
    encoded = json.dumps(
        chain.as_mapping(),
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    )
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


__all__ = [
    "LinearAxis",
    "LinearAxisChain",
    "linear_axis_chain_digest",
]
