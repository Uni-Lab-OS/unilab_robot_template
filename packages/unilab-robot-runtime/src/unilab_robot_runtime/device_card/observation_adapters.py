"""从 Preview / MoveIt / PLC 等执行器产出统一 ExecutorObservation。"""

from __future__ import annotations

import math
import time
from typing import Any

from .observation_types import ExecutorObservation, ExecutorTcpPose
from .manual_motion import _quaternion_to_euler_xyz_degrees


def observation_from_preview_arm(
    device: object,
    *,
    device_id: str,
    tcp_pose_builder: object | None = None,
) -> ExecutorObservation:
    joints_deg = list(getattr(device, "joints_deg", []) or [])
    if len(joints_deg) < 6:
        raise RuntimeError("Preview 臂关节数量不足")
    joint_refs = [f"{device_id}_joint_{index}" for index in range(1, 7)]
    joints_si = [math.radians(float(value)) for value in joints_deg[:6]]
    rail_si = getattr(device, "rail", None)
    rail_position_si = float(rail_si) if rail_si is not None else None
    tcp_pose = None
    if callable(tcp_pose_builder):
        tcp_pose = tcp_pose_builder(device, joints_deg, rail_position_si)
    busy = False
    motion = getattr(device, "_motion", None)
    if motion is not None and hasattr(motion, "locked"):
        try:
            busy = bool(motion.locked())
        except Exception:
            busy = False
    return {
        "source": "preview",
        "online": True,
        "idle": not busy,
        "stale": False,
        "observed_at": time.time(),
        "execution_fenced": False,
        "joint_positions_si": joints_si,
        "joint_refs": joint_refs,
        "rail_position_si": rail_position_si,
        "tcp_pose": tcp_pose,
    }


def observation_from_commissioning_port(port: object) -> ExecutorObservation:
    snapshot = port.commissioning_snapshot()
    joints = snapshot.joint_positions or ()
    pose = snapshot.tcp_pose
    tcp_pose: ExecutorTcpPose | None = None
    if pose is not None:
        tcp_pose = {
            "frame_ref": str(pose.frame_ref),
            "xyz_mm": [float(value) * 1000.0 for value in pose.xyz_m],
            "rotation_xyz_deg": list(
                _quaternion_to_euler_xyz_degrees(pose.orientation_xyzw)
            ),
            "xyz_m": [float(value) for value in pose.xyz_m],
            "orientation_xyzw": [float(value) for value in pose.orientation_xyzw],
        }
    return {
        "source": str(snapshot.source),
        "online": bool(snapshot.online),
        "idle": bool(snapshot.idle),
        "stale": not snapshot.is_fresh(),
        "observed_at": float(snapshot.observed_at),
        "execution_fenced": bool(snapshot.execution_fenced),
        "joint_positions_si": [float(item.position_si) for item in joints],
        "joint_refs": [str(item.joint_ref) for item in joints],
        "rail_position_si": getattr(port, "rail_position_si", None),
        "tcp_pose": tcp_pose,
    }


def observation_from_plc(*_args: Any, **_kwargs: Any) -> ExecutorObservation:
    raise NotImplementedError("PLC 观测适配器尚未实现")


def require_observation_ready(observation: ExecutorObservation, *, confirm: bool) -> None:
    if confirm is not True:
        raise ValueError("动作必须显式确认")
    if observation.get("online") is not True:
        raise RuntimeError("执行器离线")
    if observation.get("idle") is not True:
        raise RuntimeError("执行器忙碌")
    if observation.get("execution_fenced"):
        raise RuntimeError("执行器存在 Fence")
    if observation.get("stale"):
        raise RuntimeError("执行器观测过期")
    joints = observation.get("joint_positions_si") or []
    if not joints:
        raise RuntimeError("执行器关节观测不完整")


def arm_snapshot_from_observation(observation: ExecutorObservation) -> dict[str, object] | None:
    joints = observation.get("joint_positions_si") or []
    pose = observation.get("tcp_pose")
    if not joints and pose is None:
        return None
    payload: dict[str, object] = {}
    if joints:
        payload["joint_positions_si"] = [float(value) for value in joints]
    if pose is not None:
        payload["tcp_pose"] = {
            "frame_ref": str(pose.get("frame_ref", "")),
            "xyz_m": list(pose.get("xyz_m", pose.get("xyz_mm", []))),
            "orientation_xyzw": list(pose.get("orientation_xyzw", [])),
        }
    return payload
