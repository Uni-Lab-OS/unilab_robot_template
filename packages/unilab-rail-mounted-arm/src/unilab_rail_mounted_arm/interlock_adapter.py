"""配置驱动的 PLC/控制器硬件互锁观测适配器。"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Callable, Protocol

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

    def __post_init__(self) -> None:
        """拒绝字符串布尔值、空变量和非法 TTL，防止配置被 Python truthiness 放大。"""

        variables = (
            self.granted_variable,
            self.rail_permitted_variable,
            self.arm_permitted_variable,
            self.concurrent_blocked_variable,
        )
        if any(not isinstance(value, str) or not value.strip() for value in variables):
            raise ValueError("InterlockBinding 变量名必须是非空字符串")
        if not isinstance(self.hardware_enforced, bool):
            raise TypeError("InterlockBinding.hardware_enforced 必须是 bool")
        if not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("InterlockBinding.source 必须是非空字符串")
        if (
            not isinstance(self.max_age_s, (int, float))
            or isinstance(self.max_age_s, bool)
            or not 0 < self.max_age_s <= 5
        ):
            raise ValueError("InterlockBinding.max_age_s 必须位于 (0, 5]")


@dataclass(frozen=True)
class SpatialInterlockBinding:
    """部署后才可启用的空间硬件许可绑定；默认关闭。"""

    granted_variable: str
    source: str
    enabled: bool = False
    hardware_enforced: bool = False
    qualified_evidence_digest: str | None = None
    max_age_s: float = 0.5

    def __post_init__(self) -> None:
        """启用时要求真实硬件链标记和锁定的资格证据摘要。"""

        if not isinstance(self.enabled, bool) or not isinstance(
            self.hardware_enforced, bool
        ):
            raise TypeError("SpatialInterlockBinding enabled/hardware_enforced 必须是 bool")
        if not isinstance(self.granted_variable, str) or not self.granted_variable.strip():
            raise ValueError("SpatialInterlockBinding.granted_variable 必须是非空字符串")
        if not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("SpatialInterlockBinding.source 必须是非空字符串")
        if (
            not isinstance(self.max_age_s, (int, float))
            or isinstance(self.max_age_s, bool)
            or not 0 < self.max_age_s <= 5
        ):
            raise ValueError("SpatialInterlockBinding.max_age_s 必须位于 (0, 5]")
        if not self.enabled:
            return
        digest = self.qualified_evidence_digest
        if not self.hardware_enforced:
            raise ValueError("启用空间互锁必须来自经验证硬件安全链")
        if (
            not isinstance(digest, str)
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
        ):
            raise ValueError("启用空间互锁必须绑定小写 sha256 资格证据")


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
            granted = _strict_bool(
                self.port.read(self.binding.granted_variable),
                self.binding.granted_variable,
            )
            rail = _strict_bool(
                self.port.read(self.binding.rail_permitted_variable),
                self.binding.rail_permitted_variable,
            )
            arm = _strict_bool(
                self.port.read(self.binding.arm_permitted_variable),
                self.binding.arm_permitted_variable,
            )
            blocked = _strict_bool(
                self.port.read(self.binding.concurrent_blocked_variable),
                self.binding.concurrent_blocked_variable,
            )
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


class SpatiallyGuardedInterlockProvider:
    """把独立 PLC 空间许可与既有运动互斥许可做逻辑与，任一异常即拒绝。"""

    def __init__(
        self,
        *,
        base_observation: Callable[[], SafetyInterlockObservation],
        port: InterlockVariablePort,
        binding: SpatialInterlockBinding,
    ) -> None:
        """注入既有硬件安全链、空间许可节点和默认关闭的部署绑定。"""

        self._base_observation = base_observation
        self._port = port
        self.binding = binding

    def read(self) -> SafetyInterlockObservation:
        """只有两个硬件链都已知、新鲜且明确许可时才保留 motion grant。"""

        if not self.binding.enabled:
            return self._unknown(time.time(), "spatial-interlock-disabled")
        try:
            base = self._base_observation()
            spatial_granted = _strict_bool(
                self._port.read(self.binding.granted_variable),
                self.binding.granted_variable,
            )
        except Exception:  # noqa: BLE001
            return self._unknown(time.time(), "spatial-interlock-read-failed")
        observed_at = time.time()
        if not isinstance(base, SafetyInterlockObservation) or not base.is_fresh(
            observed_at
        ):
            return self._unknown(observed_at, "base-interlock-unknown-or-stale")
        hardware_enforced = (
            base.hardware_enforced and self.binding.hardware_enforced
        )
        if not hardware_enforced:
            return self._unknown(observed_at, "spatial-interlock-not-hardware-enforced")
        if not spatial_granted:
            return SafetyInterlockObservation(
                ObservationState.KNOWN,
                min(base.observed_at, observed_at),
                min(base.max_age_s, float(self.binding.max_age_s)),
                f"{base.source}+{self.binding.source}:spatial-denied",
                False,
                True,
                False,
                False,
                base.concurrent_motion_blocked,
            )
        return SafetyInterlockObservation(
            ObservationState.KNOWN,
            min(base.observed_at, observed_at),
            min(base.max_age_s, float(self.binding.max_age_s)),
            f"{base.source}+{self.binding.source}",
            base.granted,
            True,
            base.rail_motion_permitted,
            base.arm_motion_permitted,
            base.concurrent_motion_blocked,
        )

    def _unknown(self, observed_at: float, reason: str) -> SafetyInterlockObservation:
        """构造不会保留任何运动许可的 UNKNOWN 观测。"""

        return SafetyInterlockObservation(
            ObservationState.UNKNOWN,
            observed_at,
            float(self.binding.max_age_s),
            f"{self.binding.source}:{reason}",
            False,
            False,
            False,
            False,
            False,
        )


def _strict_bool(value: Any, variable: str) -> bool:
    """PLC 布尔节点必须返回真实 bool，字符串和数字一律视为读取失败。"""

    if not isinstance(value, bool):
        raise TypeError(f"互锁节点 {variable} 必须返回 bool")
    return value
