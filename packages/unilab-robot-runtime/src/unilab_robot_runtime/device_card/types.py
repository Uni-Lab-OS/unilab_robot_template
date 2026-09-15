"""机械臂设备卡片（Device Card）稳定 JSON 类型。"""

from __future__ import annotations

from typing import TypedDict


class RobotDebugTcpPose(TypedDict):
    frame_ref: str
    xyz_mm: list[float]
    rotation_xyz_deg: list[float]


class RobotDebugJointPosition(TypedDict):
    joint_ref: str
    position_deg: float


class RobotDebugPointTarget(TypedDict, total=False):
    target_ref: str
    source_point: str
    kind: str
    editable: bool
    joint_positions_deg: list[float]
    rail_position_mm: float | None
    group_ref: str
    tcp_pose: RobotDebugTcpPose


class RobotDebugCapabilities(TypedDict):
    move_target: bool
    move_pose: bool
    tcp_jog: bool
    joint_jog: bool
    controlled_stop: bool
    rail_move: bool
    composite_point_record: bool


class RobotDebugRail(TypedDict):
    position_mm: float
    travel_min_mm: float
    travel_max_mm: float


class RobotDebugSnapshot(TypedDict, total=False):
    point_set_revision: str
    source: str
    online: bool
    idle: bool
    stale: bool
    observed_at: float
    execution_fenced: bool
    active_command_id: str | None
    tcp_pose: RobotDebugTcpPose
    joint_positions: list[RobotDebugJointPosition]
    point_targets: list[RobotDebugPointTarget]
    capabilities: RobotDebugCapabilities
    rail: RobotDebugRail | None
    velocity_limit: float
    acceleration_limit: float
    vision: dict[str, object]


class RobotManualMotionResult(TypedDict):
    success: bool
    message: str
    command_id: str


class RobotTeachPointResult(TypedDict):
    target_ref: str
    target_revision: str


class RobotRailMotionResult(TypedDict):
    success: bool
    position_mm: float
