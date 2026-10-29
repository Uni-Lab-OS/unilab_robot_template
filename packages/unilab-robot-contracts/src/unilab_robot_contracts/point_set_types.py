"""PointSet v3 的稳定值对象，避免解析器承担数据定义职责。"""

from __future__ import annotations

import re
from collections.abc import Mapping
from dataclasses import dataclass

from .geometry import RigidTransform

_DIGEST = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True, slots=True)
class InstallationCalibration:
    """把设备局部坐标精确转换到机械臂基座的一版安装标定。"""

    revision: str
    digest: str
    frame_transforms: Mapping[str, RigidTransform]

    def __post_init__(self) -> None:
        """校验版本、摘要和可选设备局部坐标变换集合。"""

        if not self.revision.strip():
            raise ValueError("InstallationCalibration.revision 不能为空")
        if _DIGEST.fullmatch(self.digest.lower()) is None:
            raise ValueError("InstallationCalibration.digest 必须是 64 位 SHA-256")
        invalid = [
            frame_ref
            for frame_ref in self.frame_transforms
            if not str(frame_ref).startswith("device:")
        ]
        if invalid:
            raise ValueError(f"安装标定含非 device: 坐标系: {invalid}")


@dataclass(frozen=True, slots=True)
class ResolvedRailTarget:
    """已由精确 RailModel 校验的直接 SI 导轨目标。"""

    target_ref: str
    position_si: float
    source_chain: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class AccessMotionBlock:
    """D9-1S 封闭接近运动块的已解析几何引用。"""

    block_ref: str
    interaction_target_ref: str
    approach_target_ref: str
    entry_target_ref: str
    transit_in: tuple[str, ...]
    transit_out: tuple[str, ...]
    return_mode: str


@dataclass(frozen=True, slots=True)
class PointSetCatalogEntry:
    """作者层可调试目录中的一条关节目标，不含 AccessMotionBlock 派生点。"""

    target_ref: str
    source_point: str
    kind: str
    editable: bool
    joint_positions_si: tuple[float, ...]
    rail_position_si: float | None
    group_ref: str


@dataclass(frozen=True, slots=True)
class ResolvedPointTarget:
    """一个设备点位包解析后的机械臂、导轨和接近运动块。"""

    target_ref: str
    device_ref: str
    arm_target_ref: str | None
    rail_position_si: float | None
    access_block: AccessMotionBlock | None
    source_chain: tuple[str, ...]
    resolved_digest: str


__all__ = [
    "AccessMotionBlock",
    "InstallationCalibration",
    "PointSetCatalogEntry",
    "ResolvedPointTarget",
    "ResolvedRailTarget",
]
