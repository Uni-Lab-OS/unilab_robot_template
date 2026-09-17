"""执行器观测与点位目录快照类型（与 MoveIt commissioning 解耦）。"""

from __future__ import annotations

from typing import TypedDict


class ExecutorTcpPose(TypedDict):
    frame_ref: str
    xyz_mm: list[float]
    rotation_xyz_deg: list[float]
    xyz_m: list[float]
    orientation_xyzw: list[float]


class ExecutorObservation(TypedDict, total=False):
    source: str
    online: bool
    idle: bool
    stale: bool
    observed_at: float
    execution_fenced: bool
    joint_positions_si: list[float]
    joint_refs: list[str]
    rail_position_si: float | None
    tcp_pose: ExecutorTcpPose | None


class PointCatalogCapabilities(TypedDict):
    move_target: bool
    move_pose: bool
    tcp_jog: bool
    joint_jog: bool
    controlled_stop: bool
    rail_move: bool
    composite_point_record: bool
    point_record: bool


class PointCatalogSnapshot(TypedDict, total=False):
    point_set_revision: str
    source: str
    online: bool
    idle: bool
    stale: bool
    observed_at: float
    execution_fenced: bool
    active_command_id: str | None
    tcp_pose: dict[str, object]
    joint_positions: list[dict[str, object]]
    point_targets: list[dict[str, object]]
    capabilities: PointCatalogCapabilities
    rail: dict[str, object] | None
    velocity_limit: float
    acceleration_limit: float
    vision: dict[str, object]
    calibration: dict[str, object]
