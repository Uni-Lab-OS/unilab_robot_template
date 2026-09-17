"""PointSet 目录快照（read_point_catalog / read_debug_snapshot 共用）。"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml

from .context import ArmCardContext
from .manual_motion import _project_target, _rail_move_capable
from .observation_types import ExecutorObservation, PointCatalogSnapshot
from .types import RobotDebugRail, RobotDebugSnapshot, RobotDebugTcpPose


def read_point_catalog_from_observation(
    context: ArmCardContext,
    observation: ExecutorObservation,
    *,
    point_set_revision: str,
    catalog_port: object | None = None,
    moveit_port: object | None = None,
    calibration_meta: dict[str, object] | None = None,
) -> PointCatalogSnapshot:
    tcp_pose = _catalog_tcp_pose(observation)
    catalog_entries = _resolve_catalog_entries(context, catalog_port, moveit_port)
    catalog = [_project_target(entry, context) for entry in catalog_entries]
    rail = _catalog_rail(context, observation, catalog_port, moveit_port)
    capabilities = _catalog_capabilities(
        context,
        observation,
        catalog_port=catalog_port,
        moveit_port=moveit_port,
        rail=rail,
    )
    result: PointCatalogSnapshot = {
        "point_set_revision": point_set_revision,
        "source": str(observation.get("source", "executor")),
        "online": bool(observation.get("online")),
        "idle": bool(observation.get("idle")),
        "stale": bool(observation.get("stale")),
        "observed_at": float(observation.get("observed_at", 0.0)),
        "execution_fenced": bool(observation.get("execution_fenced")),
        "active_command_id": None,
        "tcp_pose": tcp_pose,
        "joint_positions": _catalog_joint_positions(observation),
        "point_targets": catalog,
        "capabilities": capabilities,
        "rail": rail,
    }
    if moveit_port is not None:
        result["velocity_limit"] = float(moveit_port.commissioning_velocity_limit)
        result["acceleration_limit"] = float(moveit_port.commissioning_acceleration_limit)
    if context.include_vision_in_snapshot and context.vision_snapshot is not None:
        result["vision"] = context.vision_snapshot()
    if calibration_meta is not None:
        result["calibration"] = calibration_meta
    return result


def read_point_catalog_from_moveit(
    binding: object,
    context: ArmCardContext,
) -> RobotDebugSnapshot:
    from .manual_motion import read_debug_snapshot

    return read_debug_snapshot(binding, context)


def point_set_revision_from_path(point_set_path: str | Path) -> str:
    path = Path(point_set_path)
    text = path.read_text(encoding="utf-8")
    loaded = yaml.safe_load(text)
    if not isinstance(loaded, dict):
        raise ValueError(f"{path} 必须是 PointSet YAML 对象")
    revision = str(loaded.get("revision", "")).strip()
    if not revision:
        raise ValueError(f"{path} 缺少 revision")
    digest = str(loaded.get("source_digest", ""))[:12]
    return f"{revision}@{digest}" if digest else revision


def _resolve_catalog_entries(
    context: ArmCardContext,
    catalog_port: object | None,
    moveit_port: object | None,
) -> list[Any]:
    if context.catalog_entries is not None:
        port = moveit_port or catalog_port or object()
        return list(context.catalog_entries(port))
    port = moveit_port or catalog_port
    if port is None:
        return context.resolve_catalog(object())
    return context.resolve_catalog(port)


def _catalog_tcp_pose(observation: ExecutorObservation) -> RobotDebugTcpPose:
    pose = observation.get("tcp_pose")
    if pose is None:
        return {"frame_ref": "", "xyz_mm": [], "rotation_xyz_deg": []}
    return {
        "frame_ref": str(pose.get("frame_ref", "")),
        "xyz_mm": [float(value) for value in pose.get("xyz_mm", [])],
        "rotation_xyz_deg": [float(value) for value in pose.get("rotation_xyz_deg", [])],
    }


def _catalog_joint_positions(observation: ExecutorObservation) -> list[dict[str, object]]:
    joints_si = observation.get("joint_positions_si") or []
    joint_refs = observation.get("joint_refs") or []
    items: list[dict[str, object]] = []
    for index, position_si in enumerate(joints_si):
        joint_ref = joint_refs[index] if index < len(joint_refs) else f"joint_{index + 1}"
        items.append(
            {
                "joint_ref": joint_ref,
                "position_deg": math.degrees(float(position_si)),
            }
        )
    return items


def _catalog_rail(
    context: ArmCardContext,
    observation: ExecutorObservation,
    catalog_port: object | None,
    moveit_port: object | None,
) -> RobotDebugRail | None:
    port = moveit_port or catalog_port
    if port is not None and context.rail_snapshot is not None:
        rail, _axis = context.rail_snapshot(None, port)
        if rail is not None:
            return rail
    rail_si = observation.get("rail_position_si")
    if rail_si is None:
        return None
    return {
        "position_mm": float(rail_si) * 1000.0,
        "travel_min_mm": 0.0,
        "travel_max_mm": 0.0,
    }


def _catalog_capabilities(
    context: ArmCardContext,
    observation: ExecutorObservation,
    *,
    catalog_port: object | None,
    moveit_port: object | None,
    rail: RobotDebugRail | None,
) -> dict[str, bool]:
    point_record = context.point_set_path is not None and bool(
        observation.get("joint_positions_si")
    )
    if moveit_port is not None:
        caps = moveit_port.commissioning_capabilities
        return {
            "move_target": bool(caps.move_target),
            "move_pose": bool(caps.move_pose),
            "tcp_jog": bool(caps.tcp_jog),
            "joint_jog": bool(caps.joint_jog),
            "controlled_stop": bool(caps.controlled_stop),
            "rail_move": _rail_move_capable(moveit_port, rail, None, context),
            "composite_point_record": callable(
                getattr(moveit_port, "revise_authored_point_target", None)
            )
            or point_record,
            "point_record": point_record,
        }
    return {
        "move_target": False,
        "move_pose": False,
        "tcp_jog": True,
        "joint_jog": True,
        "controlled_stop": True,
        "rail_move": rail is not None and context.move_rail is not None,
        "composite_point_record": point_record,
        "point_record": point_record,
    }
