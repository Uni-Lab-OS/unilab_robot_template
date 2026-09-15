"""Preview 机械臂运行时共享类型。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TypedDict


class MotionResult(TypedDict):
    status: str
    joints_deg: list[float]
    mode: str


class StopResult(TypedDict):
    status: str
    mode: str


@dataclass(frozen=True)
class ArmMount:
    """机械臂基座与可选导轨限位（世界坐标，米）。"""

    arm_id: str
    base_xyz: tuple[float, float, float]
    rail_limits: tuple[float, float] | None = None
