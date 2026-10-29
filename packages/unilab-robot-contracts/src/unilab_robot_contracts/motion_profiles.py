"""把运动策略从 PointSet 几何中分离的版本化配置合同。"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from typing import Any


class MotionPrimitive(str, Enum):
    """机械臂 Adapter 必须实现的最小运动原语。"""

    JOINT_PTP = "joint_ptp"
    CARTESIAN_LINEAR = "cartesian_linear"


@dataclass(frozen=True, slots=True)
class MotionProfile:
    """一条机械臂运动段的速度、容差与碰撞策略。"""

    profile_ref: str
    primitive: MotionPrimitive
    velocity_scale: float
    acceleration_scale: float
    position_tolerance_m: float
    orientation_tolerance_rad: float
    collision_check: bool

    def __post_init__(self) -> None:
        """拒绝空引用、非法比例和非正容差。"""

        if not self.profile_ref.strip():
            raise ValueError("MotionProfile.profile_ref 不能为空")
        scales = (self.velocity_scale, self.acceleration_scale)
        if any(not math.isfinite(value) or not 0.0 < value <= 1.0 for value in scales):
            raise ValueError("MotionProfile 速度和加速度比例必须位于 (0, 1]")
        tolerances = (self.position_tolerance_m, self.orientation_tolerance_rad)
        if any(not math.isfinite(value) or value <= 0.0 for value in tolerances):
            raise ValueError("MotionProfile 容差必须是有限正数")


@dataclass(frozen=True, slots=True)
class RailMotionProfile:
    """单轴导轨独立的速度、加速度和到位容差。"""

    profile_ref: str
    velocity_scale: float
    acceleration_scale: float
    settle_tolerance_m: float

    def __post_init__(self) -> None:
        """验证导轨策略不借用机械臂关节单位。"""

        if not self.profile_ref.strip():
            raise ValueError("RailMotionProfile.profile_ref 不能为空")
        if any(
            not math.isfinite(value) or not 0.0 < value <= 1.0
            for value in (self.velocity_scale, self.acceleration_scale)
        ):
            raise ValueError("RailMotionProfile 速度和加速度比例必须位于 (0, 1]")
        if not math.isfinite(self.settle_tolerance_m) or self.settle_tolerance_m <= 0.0:
            raise ValueError("RailMotionProfile.settle_tolerance_m 必须为有限正数")


@dataclass(frozen=True, slots=True)
class AccessMotionPolicy:
    """D9-1S 各几何阶段使用哪个 MotionProfile 的稳定映射。"""

    policy_ref: str
    transit_profile_ref: str
    entry_profile_ref: str
    approach_profile_ref: str
    interaction_profile_ref: str
    rail_profile_ref: str | None = None

    def __post_init__(self) -> None:
        """要求每个机械臂阶段均显式引用运动策略。"""

        required = (
            self.policy_ref,
            self.transit_profile_ref,
            self.entry_profile_ref,
            self.approach_profile_ref,
            self.interaction_profile_ref,
        )
        if any(not value.strip() for value in required):
            raise ValueError("AccessMotionPolicy 的机械臂阶段引用不得为空")


@dataclass(frozen=True, slots=True)
class MotionProfileCatalog:
    """一次激活冻结的机械臂、导轨和接近块策略集合。"""

    revision: str
    arm: Mapping[str, MotionProfile]
    rail: Mapping[str, RailMotionProfile]
    access: Mapping[str, AccessMotionPolicy]

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> MotionProfileCatalog:
        """解析唯一 v1 资产并交叉验证所有 profile 引用。"""

        if data.get("schema") != "unilab.robot-motion-profiles/v1":
            raise ValueError("motion profile schema 必须为 unilab.robot-motion-profiles/v1")
        revision = str(data.get("revision", "")).strip()
        if not revision:
            raise ValueError("MotionProfileCatalog.revision 不能为空")
        arm = {
            str(name): _arm_profile(str(name), _mapping(value, f"arm.{name}"))
            for name, value in _mapping(data.get("arm"), "arm").items()
        }
        rail = {
            str(name): _rail_profile(str(name), _mapping(value, f"rail.{name}"))
            for name, value in _mapping(data.get("rail", {}), "rail").items()
        }
        access = {
            str(name): _access_policy(str(name), _mapping(value, f"access.{name}"))
            for name, value in _mapping(data.get("access"), "access").items()
        }
        for policy in access.values():
            refs = (
                policy.transit_profile_ref,
                policy.entry_profile_ref,
                policy.approach_profile_ref,
                policy.interaction_profile_ref,
            )
            missing = [profile_ref for profile_ref in refs if profile_ref not in arm]
            if missing:
                raise ValueError(f"{policy.policy_ref} 引用缺失 arm profile: {missing}")
            if policy.rail_profile_ref is not None and policy.rail_profile_ref not in rail:
                raise ValueError(
                    f"{policy.policy_ref} 引用缺失 rail profile: {policy.rail_profile_ref}"
                )
        return cls(revision, arm, rail, access)


def _arm_profile(name: str, data: Mapping[str, Any]) -> MotionProfile:
    """把一个机械臂策略 YAML 对象转换为类型化值。"""

    return MotionProfile(
        name,
        MotionPrimitive(str(data["primitive"])),
        float(data["velocity_scale"]),
        float(data["acceleration_scale"]),
        float(data["position_tolerance_m"]),
        float(data["orientation_tolerance_rad"]),
        bool(data.get("collision_check", True)),
    )


def _rail_profile(name: str, data: Mapping[str, Any]) -> RailMotionProfile:
    """把一个导轨策略 YAML 对象转换为类型化值。"""

    return RailMotionProfile(
        name,
        float(data["velocity_scale"]),
        float(data["acceleration_scale"]),
        float(data["settle_tolerance_m"]),
    )


def _access_policy(name: str, data: Mapping[str, Any]) -> AccessMotionPolicy:
    """把 D9-1S 阶段角色映射转换为类型化值。"""

    rail_ref = data.get("rail_profile_ref")
    return AccessMotionPolicy(
        name,
        str(data["transit_profile_ref"]),
        str(data["entry_profile_ref"]),
        str(data["approach_profile_ref"]),
        str(data["interaction_profile_ref"]),
        None if rail_ref is None else str(rail_ref),
    )


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    """要求运动策略字段为对象。"""

    if not isinstance(value, Mapping):
        raise TypeError(f"{field} 必须是对象")
    return value


__all__ = [
    "AccessMotionPolicy",
    "MotionPrimitive",
    "MotionProfile",
    "MotionProfileCatalog",
    "RailMotionProfile",
]
