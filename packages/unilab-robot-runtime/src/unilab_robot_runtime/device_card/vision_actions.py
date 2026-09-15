"""机械臂卡片 vision 标定动作（template 通用实现）。"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from unilab_robot_runtime.vision_calibration.paths import VisionCalibrationPaths
from unilab_robot_runtime.vision_calibration.store import (
    build_card_vision_snapshot,
    record_camera_extrinsic_stub,
    record_marker_for_warehouse,
    record_tcp_calibration_stub,
    warehouse_from_group_ref,
)

from .manual_motion import _port, _quaternion_to_euler_xyz_degrees


def card_vision_snapshot(paths: VisionCalibrationPaths) -> dict[str, object]:
    return build_card_vision_snapshot(paths)


def calibrate_camera_extrinsic(
    binding: object,
    paths: VisionCalibrationPaths,
    *,
    confirm: bool,
) -> dict[str, object]:
    _require_commissioning_ready(binding, confirm=confirm)
    return record_camera_extrinsic_stub(paths)


def calibrate_tcp(
    binding: object,
    paths: VisionCalibrationPaths,
    *,
    confirm: bool,
) -> dict[str, object]:
    port = _require_commissioning_ready(binding, confirm=confirm)
    snapshot = port.commissioning_snapshot()
    pose = snapshot.tcp_pose
    tcp_pose = None
    if pose is not None:
        tcp_pose = {
            "frame_ref": str(pose.frame_ref),
            "xyz_mm": [float(value) * 1000.0 for value in pose.xyz_m],
            "rotation_xyz_deg": list(
                _quaternion_to_euler_xyz_degrees(pose.orientation_xyzw)
            ),
        }
    return record_tcp_calibration_stub(paths, tcp_pose=tcp_pose)


def record_marker(
    binding: object,
    paths: VisionCalibrationPaths,
    *,
    warehouse_ref: str,
    confirm: bool,
) -> dict[str, object]:
    port = _require_commissioning_ready(binding, confirm=confirm)
    normalized = warehouse_from_group_ref(warehouse_ref)
    snapshot = port.commissioning_snapshot()
    arm_snapshot = _arm_snapshot(snapshot)
    return record_marker_for_warehouse(
        normalized,
        paths,
        arm_snapshot=arm_snapshot,
    )


def bind_record_vision(
    target_ref: str,
    marker_ref: str | None,
    paths: VisionCalibrationPaths,
) -> None:
    from unilab_robot_runtime.vision_calibration.store import (
        bind_point_to_marker,
        default_marker_for_warehouse,
        warehouse_from_group_ref,
    )

    resolved_marker = str(marker_ref or "").strip()
    if not resolved_marker:
        group_ref = target_ref.split(".", 1)[0]
        resolved_marker = str(
            default_marker_for_warehouse(
                warehouse_from_group_ref(group_ref),
                paths,
            )["marker_ref"]
        )
    bind_point_to_marker(target_ref, resolved_marker, paths)


def _arm_snapshot(snapshot: object) -> dict[str, Any] | None:
    joints = getattr(snapshot, "joint_positions", None) or ()
    pose = getattr(snapshot, "tcp_pose", None)
    if not joints and pose is None:
        return None
    payload: dict[str, Any] = {}
    if joints:
        payload["joint_positions_si"] = [
            float(item.position_si) for item in joints
        ]
    if pose is not None:
        payload["tcp_pose"] = {
            "frame_ref": str(pose.frame_ref),
            "xyz_m": [float(value) for value in pose.xyz_m],
            "orientation_xyzw": [float(value) for value in pose.orientation_xyzw],
        }
    return payload


def _require_commissioning_ready(binding: object, *, confirm: bool) -> object:
    if confirm is not True:
        raise ValueError("视觉校准动作必须显式确认")
    port = _port(binding)
    snapshot = port.commissioning_snapshot()
    if (
        not snapshot.is_fresh()
        or not snapshot.online
        or not snapshot.idle
        or snapshot.execution_fenced
    ):
        raise RuntimeError("机械臂快照离线、忙碌、过期、不完整或存在 Fence")
    return port
