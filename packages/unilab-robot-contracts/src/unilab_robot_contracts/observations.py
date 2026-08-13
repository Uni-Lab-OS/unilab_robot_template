"""机械臂、导轨与硬件许可互相独立的观测合同。"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum


class ObservationState(str, Enum):
    """观测是否包含可用于准入的确定事实。"""

    KNOWN = "known"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class _TimedObservation:
    """观测接收时间与新鲜度的共享实现。"""

    state: ObservationState
    observed_at: float
    max_age_s: float
    source: str

    def is_fresh(self, now: float | None = None) -> bool:
        """判断观测是否已知、未来自未来且未超过 TTL。"""

        checked_at = time.time() if now is None else now
        age = checked_at - self.observed_at
        return (
            self.state is ObservationState.KNOWN
            and self.max_age_s > 0
            and 0 <= age <= self.max_age_s
        )


@dataclass(frozen=True)
class EndEffectorObservation(_TimedObservation):
    """夹爪/末端负载见证；不承担 Site 在位职责。"""

    holding_payload: bool | None


@dataclass(frozen=True)
class RailStateObservation(_TimedObservation):
    """单轴导轨位置、运动和稳定事实。"""

    position: float | None
    moving: bool | None
    settled: bool | None
    target_ref: str | None = None
    completed_command_id: str | None = None


@dataclass(frozen=True)
class SafetyInterlockObservation(_TimedObservation):
    """硬件侧互斥许可镜像；普通遥测不得设置 hardware_enforced。"""

    granted: bool
    hardware_enforced: bool
    rail_motion_permitted: bool
    arm_motion_permitted: bool
    concurrent_motion_blocked: bool
