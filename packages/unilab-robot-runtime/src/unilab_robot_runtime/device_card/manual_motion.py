"""机械臂设备卡片离散维护动作（template 通用实现）。"""

from __future__ import annotations

import math
import uuid
from typing import Any

from .context import ArmCardContext
from .types import (
    RobotDebugPointTarget,
    RobotDebugRail,
    RobotDebugSnapshot,
    RobotDebugTcpPose,
    RobotManualMotionResult,
    RobotRailMotionResult,
    RobotTeachPointResult,
)


def read_debug_snapshot(binding: object, context: ArmCardContext) -> RobotDebugSnapshot:
    port = _port(binding)
    snapshot = port.commissioning_snapshot()
    pose = snapshot.tcp_pose
    tcp_pose: RobotDebugTcpPose = {
        "frame_ref": "",
        "xyz_mm": [],
        "rotation_xyz_deg": [],
    }
    if pose is not None:
        tcp_pose = {
            "frame_ref": str(pose.frame_ref),
            "xyz_mm": [float(value) * 1000.0 for value in pose.xyz_m],
            "rotation_xyz_deg": list(
                _quaternion_to_euler_xyz_degrees(pose.orientation_xyzw)
            ),
        }
    catalog = [_project_target(entry, context) for entry in context.resolve_catalog(port)]
    capabilities = port.commissioning_capabilities
    rail: RobotDebugRail | None = None
    rail_axis = None
    if context.rail_snapshot is not None:
        rail, rail_axis = context.rail_snapshot(binding, port)
    elif getattr(port, "rail_position_si", None) is not None:
        rail_limits = getattr(port, "rail_limits_si", None)
        if rail_limits is not None:
            rail = {
                "position_mm": float(port.rail_position_si) * 1000.0,
                "travel_min_mm": float(rail_limits[0]) * 1000.0,
                "travel_max_mm": float(rail_limits[1]) * 1000.0,
            }
    result: RobotDebugSnapshot = {
        "point_set_revision": str(port.commissioning_target_revision or ""),
        "source": str(snapshot.source),
        "online": bool(snapshot.online),
        "idle": bool(snapshot.idle),
        "stale": not snapshot.is_fresh(),
        "observed_at": float(snapshot.observed_at),
        "execution_fenced": bool(snapshot.execution_fenced),
        "active_command_id": snapshot.active_command_id,
        "tcp_pose": tcp_pose,
        "joint_positions": [
            {
                "joint_ref": str(item.joint_ref),
                "position_deg": math.degrees(float(item.position_si)),
            }
            for item in (snapshot.joint_positions or ())
        ],
        "point_targets": catalog,
        "capabilities": {
            "move_target": bool(capabilities.move_target),
            "move_pose": bool(capabilities.move_pose),
            "tcp_jog": bool(capabilities.tcp_jog),
            "joint_jog": bool(capabilities.joint_jog),
            "controlled_stop": bool(capabilities.controlled_stop),
            "rail_move": _rail_move_capable(port, rail, rail_axis, context),
            "composite_point_record": callable(
                getattr(port, "revise_authored_point_target", None)
            )
            or context.point_set_path is not None,
            "point_record": context.point_set_path is not None
            or callable(getattr(port, "revise_authored_joint_target", None)),
        },
        "rail": rail,
        "velocity_limit": float(port.commissioning_velocity_limit),
        "acceleration_limit": float(port.commissioning_acceleration_limit),
    }
    if context.include_vision_in_snapshot and context.vision_snapshot is not None:
        result["vision"] = context.vision_snapshot()
    return result


def jog_joint_once(
    binding: object,
    context: ArmCardContext,
    *,
    device_id: str,
    monotonic_sequence: int,
    joint_ref: str,
    direction: str,
    step_deg: float,
) -> RobotManualMotionResult:
    from unilab_robot_contracts import JointJogCommand, MotionDirection

    normalized_ref = str(joint_ref).strip()
    if not normalized_ref:
        raise ValueError("关节 Jog 必须包含稳定 joint_ref")
    step = float(step_deg)
    if not math.isfinite(step) or step <= 0.0:
        raise ValueError("关节 Jog 单步必须为正的有限数")
    if context.max_joint_jog_deg is not None and step > context.max_joint_jog_deg:
        raise ValueError(f"关节 Jog 单步不得超过 {context.max_joint_jog_deg} 度")
    port = _port(binding)
    scale = _motion_scale(port)
    prefix = context.boot_id_prefix
    command = JointJogCommand(
        command_id=f"card-joint-jog-{uuid.uuid4()}",
        hardware_profile_digest=str(port.hardware_profile_digest),
        source_boot_id=f"{prefix}:{device_id}:card-joint-jog",
        monotonic_sequence=monotonic_sequence,
        motion_profile_ref=context.default_motion_profile_ref(port),
        velocity_scale=scale,
        acceleration_scale=scale,
        joint_ref=normalized_ref,
        direction=MotionDirection(str(direction).strip().lower()),
        step_si=math.radians(step),
    )
    return _execute(binding, command, owner=f"{prefix}:{device_id}:card-joint-jog")


def jog_tcp_once(
    binding: object,
    context: ArmCardContext,
    *,
    device_id: str,
    monotonic_sequence: int,
    axis: str,
    direction: str,
    step: float,
    frame_ref: str,
) -> RobotManualMotionResult:
    from unilab_robot_contracts import MotionDirection, TcpAxis, TcpJogCommand

    normalized_axis = TcpAxis(str(axis).strip().lower())
    normalized_step = float(step)
    if not math.isfinite(normalized_step) or normalized_step <= 0.0:
        raise ValueError("TCP Jog 单步必须为正的有限数")
    step_si = (
        math.radians(normalized_step)
        if normalized_axis.rotational
        else normalized_step / 1000.0
    )
    port = _port(binding)
    scale = _motion_scale(port)
    prefix = context.boot_id_prefix
    command = TcpJogCommand(
        command_id=f"card-tcp-jog-{uuid.uuid4()}",
        hardware_profile_digest=str(port.hardware_profile_digest),
        source_boot_id=f"{prefix}:{device_id}:card-tcp-jog",
        monotonic_sequence=monotonic_sequence,
        motion_profile_ref=context.default_motion_profile_ref(port),
        velocity_scale=scale,
        acceleration_scale=scale,
        frame_ref=str(frame_ref).strip(),
        axis=normalized_axis,
        direction=MotionDirection(str(direction).strip().lower()),
        step_si=step_si,
    )
    return _execute(binding, command, owner=f"{prefix}:{device_id}:card-tcp-jog")


def teach_point_from_current(
    binding: object | None,
    context: ArmCardContext | None = None,
    *,
    target_ref: str,
    confirm: bool,
    observation: object | None = None,
) -> RobotTeachPointResult:
    if context is not None and context.point_set_path is not None:
        from .observation_adapters import observation_from_commissioning_port
        from .observation_types import ExecutorObservation
        from .point_record_service import teach_joint_target_from_observation

        obs: ExecutorObservation
        if observation is not None:
            obs = observation  # type: ignore[assignment]
        elif binding is not None:
            obs = observation_from_commissioning_port(_port(binding))
        else:
            raise RuntimeError("示教落盘缺少执行器观测")
        return teach_joint_target_from_observation(
            obs,
            point_set_path=context.point_set_path,
            target_ref=target_ref,
            joint_count=context.joint_count,
            confirm=confirm,
        )
    if binding is None:
        raise RuntimeError("MoveIt 调试绑定未就绪")
    if confirm is not True:
        raise ValueError("点位设置必须显式确认")
    normalized = str(target_ref).strip()
    if not normalized:
        raise ValueError("点位设置必须包含稳定 target_ref")
    port = _port(binding)
    snapshot = port.commissioning_snapshot()
    if (
        not snapshot.is_fresh()
        or snapshot.online is not True
        or snapshot.idle is not True
        or snapshot.execution_fenced
        or not snapshot.joint_positions
    ):
        raise RuntimeError("机械臂快照离线、忙碌、过期、不完整或存在 Fence")
    result = port.revise_authored_joint_target(
        normalized,
        tuple(item.position_si for item in snapshot.joint_positions),
    )
    return {
        "target_ref": str(result["target_ref"]),
        "target_revision": str(result["target_revision"]),
    }


def move_rail_to_position(
    binding: object,
    context: ArmCardContext,
    *,
    position_mm: float,
) -> RobotRailMotionResult:
    position = float(position_mm)
    if not math.isfinite(position):
        raise ValueError("导轨绝对位置必须是有限数")
    if context.move_rail is not None:
        return context.move_rail(binding, position)
    port = _port(binding)
    mover = getattr(port, "move_rail_to_si", None)
    if not callable(mover):
        raise RuntimeError("当前机械臂设备形态未装配可控导轨")
    mover(position / 1000.0)
    return {"success": True, "position_mm": position}


def record_current_point(
    binding: object | None,
    context: ArmCardContext,
    *,
    target_ref: str,
    expected_revision: str | None,
    confirm: bool,
    include_vision: bool = False,
    marker_ref: str | None = None,
    observation: object | None = None,
) -> RobotTeachPointResult:
    if confirm is not True:
        raise ValueError("记录当前位置必须显式确认")
    normalized = str(target_ref).strip()
    if not normalized:
        raise ValueError("记录当前位置必须包含稳定 target_ref")
    if context.point_set_path is not None:
        from .observation_adapters import observation_from_commissioning_port
        from .observation_types import ExecutorObservation
        from .point_record_service import teach_joint_target_from_observation

        obs: ExecutorObservation
        if observation is not None:
            obs = observation  # type: ignore[assignment]
        elif binding is not None:
            obs = observation_from_commissioning_port(_port(binding))
        else:
            raise RuntimeError("记录落盘缺少执行器观测")
        result = teach_joint_target_from_observation(
            obs,
            point_set_path=context.point_set_path,
            target_ref=normalized,
            joint_count=context.joint_count,
            confirm=confirm,
            expected_revision=expected_revision,
            rail_position_si=obs.get("rail_position_si"),
        )
    else:
        if binding is None:
            raise RuntimeError("MoveIt 调试绑定未就绪")
        port = _port(binding)
        snapshot = port.commissioning_snapshot()
        if (
            not snapshot.is_fresh()
            or not snapshot.online
            or not snapshot.idle
            or snapshot.execution_fenced
            or not snapshot.joint_positions
        ):
            raise RuntimeError("机械臂快照离线、忙碌、过期、不完整或存在 Fence")
        reviser = getattr(port, "revise_authored_point_target", None)
        if not callable(reviser):
            raise RuntimeError("当前调试端口不支持 PointSet v3 复合点位写回")
        legacy = reviser(
            normalized,
            tuple(item.position_si for item in snapshot.joint_positions),
            getattr(port, "rail_position_si", None),
            expected_revision=expected_revision,
        )
        result = {
            "target_ref": str(legacy["target_ref"]),
            "target_revision": str(legacy["target_revision"]),
        }
    if include_vision and context.on_record_with_vision is not None:
        context.on_record_with_vision(normalized, marker_ref)
    return result


def _project_target(entry: Any, context: ArmCardContext) -> RobotDebugPointTarget:
    if context.project_target is not None:
        return context.project_target(entry)
    target_ref = str(getattr(entry, "target_ref", ""))
    source_point = str(getattr(entry, "source_point", "")).strip()
    joint_positions_si = getattr(entry, "joint_positions_si", ())
    rail_position_si = getattr(entry, "rail_position_si", None)
    return {
        "target_ref": target_ref,
        "source_point": source_point or _display_source_point(target_ref),
        "kind": str(getattr(entry, "kind", "")),
        "editable": bool(getattr(entry, "editable", False)),
        "joint_positions_deg": [math.degrees(float(value)) for value in joint_positions_si],
        "rail_position_mm": (
            float(rail_position_si) * 1000.0 if rail_position_si is not None else None
        ),
        "group_ref": str(getattr(entry, "group_ref", "")),
    }


def _rail_move_capable(
    port: object,
    rail: RobotDebugRail | None,
    rail_axis: object | None,
    context: ArmCardContext,
) -> bool:
    if context.move_rail is not None:
        return rail is not None
    if rail is not None and callable(getattr(port, "move_rail_to_si", None)):
        return True
    if rail_axis is not None and callable(getattr(rail_axis, "move_to_si", None)):
        return True
    return False


def _execute(binding: object, command: object, *, owner: str) -> RobotManualMotionResult:
    from unilab_robot_contracts import CommandState

    session = binding.open_maintenance_session(owner)
    result = session.execute(command)
    if not result.state.terminal:
        raise RuntimeError(result.message)
    session.close()
    if result.state is not CommandState.SUCCEEDED:
        raise RuntimeError(result.message)
    return {
        "success": True,
        "message": str(result.message),
        "command_id": str(result.command_id),
    }


def _port(binding: object) -> object:
    port = getattr(binding, "commissioning_port", None)
    if port is None:
        raise RuntimeError("MoveIt 调试绑定缺少 commissioning_port")
    return port


def _motion_scale(port: object) -> float:
    return min(
        float(port.commissioning_velocity_limit),
        float(port.commissioning_acceleration_limit),
    )


def _display_source_point(target_ref: str) -> str:
    parts = [part for part in str(target_ref).split(".") if part]
    if not parts:
        return str(target_ref).strip()
    last = parts[-1]
    if last in {"interaction_seed", "interaction", "entry"} and len(parts) >= 2:
        return parts[-2]
    return last


def _quaternion_to_euler_xyz_degrees(
    orientation_xyzw: tuple[float, ...] | list[float],
) -> tuple[float, float, float]:
    x, y, z, w = (float(value) for value in orientation_xyzw)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = (
        math.copysign(math.pi / 2.0, sin_pitch)
        if abs(sin_pitch) >= 1.0
        else math.asin(sin_pitch)
    )
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))
