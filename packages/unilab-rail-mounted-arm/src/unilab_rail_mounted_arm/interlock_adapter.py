"""配置驱动的 PLC/控制器硬件互锁观测适配器。"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Protocol

from unilab_robot_contracts import ObservationState, SafetyInterlockObservation


class InterlockVariablePort(Protocol):
    """读取硬件互锁镜像的最小端口。"""

    def read(self, variable: str) -> Any:
        """读取一个由部署配置指定的节点。"""


@dataclass(frozen=True)
class InterlockBinding:
    """原始节点名及其是否来源于经验证硬件安全链。"""

    granted_variable: str
    rail_permitted_variable: str
    arm_permitted_variable: str
    concurrent_blocked_variable: str
    hardware_enforced: bool
    source: str
    max_age_s: float = 0.5


class ConfiguredInterlockProvider:
    """每次读取四个独立事实，任一失败都返回 unknown。"""

    def __init__(
        self, *, port: InterlockVariablePort, binding: InterlockBinding
    ) -> None:
        """注入变量 port 与部署绑定；不从普通位置遥测推断许可。"""

        self.port = port
        self.binding = binding

    def read(self) -> SafetyInterlockObservation:
        """返回同一次读取周期的硬件互锁观测。"""

        observed_at = time.time()
        try:
            granted = bool(self.port.read(self.binding.granted_variable))
            rail = bool(self.port.read(self.binding.rail_permitted_variable))
            arm = bool(self.port.read(self.binding.arm_permitted_variable))
            blocked = bool(self.port.read(self.binding.concurrent_blocked_variable))
        except Exception:  # noqa: BLE001
            return SafetyInterlockObservation(
                ObservationState.UNKNOWN,
                observed_at,
                self.binding.max_age_s,
                self.binding.source,
                False,
                self.binding.hardware_enforced,
                False,
                False,
                False,
            )
        return SafetyInterlockObservation(
            ObservationState.KNOWN,
            observed_at,
            self.binding.max_age_s,
            self.binding.source,
            granted,
            self.binding.hardware_enforced,
            rail,
            arm,
            blocked,
        )
