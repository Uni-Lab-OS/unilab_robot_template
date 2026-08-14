"""部署硬件配置（HardwareProfile）及生产门禁。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


class BackendKind(str, Enum):
    """机械臂执行传输类型；不改变公开动作名。"""

    PLC = "plc"
    TCP_SDK = "tcp_sdk"
    MOVEIT = "moveit"


class DeploymentMode(str, Enum):
    """部署用途；维护和仿真不得冒充生产。"""

    PRODUCTION = "production"
    MAINTENANCE = "maintenance"
    SIMULATION = "simulation"


class InterlockMode(str, Enum):
    """硬件互锁证据类型。"""

    VALIDATED_HARDWARE_INTERLOCK = "validated_hardware_interlock"
    OBSERVED_ONLY = "observed_only"
    SIMULATION = "simulation"


@dataclass(frozen=True)
class HardwareProfile:
    """原子锁定执行后端、物理端点和安全证据的部署配置。"""

    profile_id: str
    digest: str
    mode: DeploymentMode
    backend: BackendKind
    endpoint_ids: frozenset[str]
    interlock_mode: InterlockMode
    commissioning_velocity_limit: float
    commissioning_acceleration_limit: float
    joint_state_stale_after_s: float = 1.0
    commissioning_joint_completion_tolerance_si: float = 0.002

    def __post_init__(self) -> None:
        """校验生产配置不能使用普通遥测或仿真许可。

        参数：无。返回：无。异常：字段缺失或生产互锁不合规时抛出 ``ValueError``。
        """

        if (
            not self.profile_id.strip()
            or not self.digest.strip()
            or not self.endpoint_ids
        ):
            raise ValueError(
                "HardwareProfile 必须包含 identity、digest 与 endpoint_ids"
            )
        if (
            self.mode is DeploymentMode.PRODUCTION
            and self.interlock_mode is not InterlockMode.VALIDATED_HARDWARE_INTERLOCK
        ):
            raise ValueError("production profile 必须使用 validated_hardware_interlock")
        for name, value in (
            ("commissioning_velocity_limit", self.commissioning_velocity_limit),
            ("commissioning_acceleration_limit", self.commissioning_acceleration_limit),
        ):
            if not math.isfinite(value) or not 0.0 < value <= 0.30:
                raise ValueError(f"HardwareProfile.{name} 必须位于 (0, 0.30]")
        if (
            not math.isfinite(self.joint_state_stale_after_s)
            or not 0.5 <= self.joint_state_stale_after_s <= 5.0
        ):
            raise ValueError(
                "HardwareProfile.joint_state_stale_after_s 必须位于 [0.5, 5.0]"
            )
        if (
            not math.isfinite(self.commissioning_joint_completion_tolerance_si)
            or not 0.0 < self.commissioning_joint_completion_tolerance_si <= 0.01
        ):
            raise ValueError(
                "HardwareProfile.commissioning_joint_completion_tolerance_si "
                "必须位于 (0, 0.01]"
            )
