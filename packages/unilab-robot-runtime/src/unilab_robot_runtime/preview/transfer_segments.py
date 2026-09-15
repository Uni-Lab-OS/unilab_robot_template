"""六段 pick-place 路径编排（hover/down/lift/carry/lower/retreat）。"""

from __future__ import annotations

from collections.abc import Sequence

from .protocols import PreviewKinematics
from .types import ArmMount


def build_pick_place_segments(
    kinematics: PreviewKinematics,
    mount: ArmMount,
    source_xyz: Sequence[float],
    target_xyz: Sequence[float],
    *,
    approach_source: Sequence[float],
    approach_target: Sequence[float],
    rail_y: float = 0.0,
    seed: Sequence[float] | None = None,
    yaw_deg: float = 90.0,
) -> tuple[
    list[float],
    list[list[float]],
    list[list[float]],
    list[list[float]],
    list[list[float]],
    list[list[float]],
]:
    """构建标准六段搬运路径。"""

    start_seed = list(seed or kinematics.home_deg)
    hover = kinematics.ik_tcp(
        mount,
        approach_source,
        seed=start_seed,
        yaw_deg=yaw_deg,
        rail_y=rail_y,
    )
    down = kinematics.linear_path(mount, hover, source_xyz, rail_y=rail_y)
    lift = kinematics.linear_path(mount, down[-1], approach_source, rail_y=rail_y)
    carry = kinematics.carry_path(mount, lift[-1], approach_target, rail_y=rail_y)
    lower = kinematics.linear_path(mount, carry[-1], target_xyz, rail_y=rail_y)
    retreat = kinematics.linear_path(mount, lower[-1], approach_target, rail_y=rail_y)
    return hover, down, lift, carry, lower, retreat
