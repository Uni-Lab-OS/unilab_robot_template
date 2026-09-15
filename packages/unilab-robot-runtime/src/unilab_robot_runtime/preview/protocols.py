"""Preview 运动学协议。"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol, runtime_checkable

from .types import ArmMount


@runtime_checkable
class PreviewKinematics(Protocol):
    """L1 型号插件必须实现的解析 FK/IK 与路径规划。"""

    home_deg: tuple[float, ...]

    def fk_tcp(
        self,
        mount: ArmMount,
        q_deg: Sequence[float],
        rail_y: float = 0.0,
    ) -> object: ...

    def plate_pose(
        self,
        mount: ArmMount,
        q_deg: Sequence[float],
        rail_y: float = 0.0,
    ) -> object: ...

    def ik_tcp(
        self,
        mount: ArmMount,
        xyz: Sequence[float],
        *,
        seed: Sequence[float] | None = None,
        yaw_deg: float = 0.0,
        rail_y: float = 0.0,
    ) -> list[float]: ...

    def linear_path(
        self,
        mount: ArmMount,
        start_q: Sequence[float],
        target_xyz: Sequence[float],
        spacing: float = 0.012,
        rail_y: float = 0.0,
    ) -> list[list[float]]: ...

    def carry_path(
        self,
        mount: ArmMount,
        start_q: Sequence[float],
        target_xyz: Sequence[float],
        rail_y: float = 0.0,
    ) -> list[list[float]]: ...
