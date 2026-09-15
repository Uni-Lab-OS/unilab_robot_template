"""InstallationCalibration 与 vision_registry 通用读写。"""

from __future__ import annotations

from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from .paths import VisionCalibrationPaths


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def device_frame_ref(warehouse_ref: str) -> str:
    normalized = str(warehouse_ref).strip()
    if not normalized:
        raise ValueError("warehouse_ref 不能为空")
    return f"device:{normalized}"


def default_marker_ref(warehouse_ref: str) -> str:
    return f"{str(warehouse_ref).strip()}/default"


def warehouse_from_group_ref(group_ref: str) -> str:
    normalized = str(group_ref).strip()
    if not normalized:
        raise ValueError("group_ref 不能为空")
    return normalized


def _dump_yaml(document: Mapping[str, Any]) -> str:
    return yaml.safe_dump(dict(document), sort_keys=False, allow_unicode=True)


def load_calibration(paths: VisionCalibrationPaths) -> dict[str, Any]:
    target = paths.calibration_path
    document = yaml.safe_load(target.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"{target} 必须是映射")
    return document


def save_calibration(document: Mapping[str, Any], paths: VisionCalibrationPaths) -> None:
    target = paths.calibration_path
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(_dump_yaml(document), encoding="utf-8", newline="\n")


def build_vision_registry_document(paths: VisionCalibrationPaths) -> dict[str, Any]:
    return {
        "schema": "unilab.robot-qualifications/v1",
        "revision": paths.registry_revision,
        "state": "pending-vision-calibration",
        "vision_registry": {
            "camera_extrinsic": {
                "state": "pending",
                "camera_id": None,
                "intrinsics_revision": None,
                "hand_eye_revision": None,
                "updated_at": None,
                "evidence_refs": [],
            },
            "tcp_calibration": {
                "state": "pending",
                "tool_context_ref": paths.tool_context_ref,
                "updated_at": None,
                "evidence_refs": [],
            },
            "markers": {},
            "point_bindings": {},
        },
    }


def load_vision_registry(paths: VisionCalibrationPaths) -> dict[str, Any]:
    target = paths.registry_path
    if not target.exists():
        return build_vision_registry_document(paths)
    document = yaml.safe_load(target.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"{target} 必须是映射")
    if "vision_registry" not in document:
        document = build_vision_registry_document(paths)
    return document


def save_vision_registry(document: Mapping[str, Any], paths: VisionCalibrationPaths) -> None:
    target = paths.registry_path
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(_dump_yaml(document), encoding="utf-8", newline="\n")


def _vision_registry(document: Mapping[str, Any]) -> dict[str, Any]:
    vision = document.get("vision_registry", {})
    if not isinstance(vision, dict):
        raise ValueError("vision_registry 必须是映射")
    return vision


def list_markers(
    paths: VisionCalibrationPaths,
    registry: Mapping[str, Any] | None = None,
) -> list[dict[str, Any]]:
    document = registry if registry is not None else load_vision_registry(paths)
    vision = _vision_registry(document)
    markers_raw = vision.get("markers", {})
    if not isinstance(markers_raw, dict):
        return []
    items: list[dict[str, Any]] = []
    for warehouse_ref, value in markers_raw.items():
        if not isinstance(value, dict):
            continue
        marker_ref = str(value.get("marker_ref") or default_marker_ref(warehouse_ref))
        items.append(
            {
                "warehouse_ref": str(warehouse_ref),
                "marker_ref": marker_ref,
                "label": str(value.get("label") or warehouse_ref),
                "device_frame_ref": str(
                    value.get("device_frame_ref") or device_frame_ref(warehouse_ref)
                ),
                "recorded_at": value.get("recorded_at"),
            }
        )
    return sorted(items, key=lambda item: str(item["marker_ref"]))


def default_marker_for_warehouse(
    warehouse_ref: str,
    paths: VisionCalibrationPaths,
    registry: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    normalized = str(warehouse_ref).strip()
    document = registry if registry is not None else load_vision_registry(paths)
    vision = _vision_registry(document)
    markers_raw = vision.get("markers", {})
    if isinstance(markers_raw, dict) and normalized in markers_raw:
        item = markers_raw[normalized]
        if isinstance(item, dict):
            return {
                "warehouse_ref": normalized,
                "marker_ref": str(item.get("marker_ref") or default_marker_ref(normalized)),
                "label": str(item.get("label") or normalized),
                "device_frame_ref": str(
                    item.get("device_frame_ref") or device_frame_ref(normalized)
                ),
                "recorded_at": item.get("recorded_at"),
            }
    return {
        "warehouse_ref": normalized,
        "marker_ref": default_marker_ref(normalized),
        "label": f"{normalized} 默认 marker",
        "device_frame_ref": device_frame_ref(normalized),
        "recorded_at": None,
    }


def bind_point_to_marker(
    target_ref: str,
    marker_ref: str,
    paths: VisionCalibrationPaths,
) -> dict[str, Any]:
    normalized_target = str(target_ref).strip()
    normalized_marker = str(marker_ref).strip()
    if not normalized_target or not normalized_marker:
        raise ValueError("target_ref 与 marker_ref 均不能为空")
    document = load_vision_registry(paths)
    vision = _vision_registry(document)
    bindings = vision.setdefault("point_bindings", {})
    if not isinstance(bindings, dict):
        raise ValueError("vision_registry.point_bindings 必须是映射")
    bindings[normalized_target] = normalized_marker
    save_vision_registry(document, paths)
    return {
        "target_ref": normalized_target,
        "marker_ref": normalized_marker,
        "registry_revision": str(document.get("revision", paths.registry_revision)),
    }


def record_camera_extrinsic_stub(paths: VisionCalibrationPaths) -> dict[str, Any]:
    document = load_vision_registry(paths)
    vision = _vision_registry(document)
    block = vision.setdefault("camera_extrinsic", {})
    if not isinstance(block, dict):
        raise ValueError("camera_extrinsic 必须是映射")
    block.update({"state": "recorded", "updated_at": utc_now_iso()})
    save_vision_registry(document, paths)
    return {
        "state": str(block.get("state", "recorded")),
        "updated_at": str(block.get("updated_at", "")),
        "registry_revision": str(document.get("revision", paths.registry_revision)),
    }


def record_tcp_calibration_stub(
    paths: VisionCalibrationPaths,
    *,
    tcp_pose: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    document = load_vision_registry(paths)
    vision = _vision_registry(document)
    block = vision.setdefault("tcp_calibration", {})
    if not isinstance(block, dict):
        raise ValueError("tcp_calibration 必须是映射")
    payload: dict[str, Any] = {
        "state": "recorded",
        "updated_at": utc_now_iso(),
        "tool_context_ref": paths.tool_context_ref,
    }
    if tcp_pose is not None:
        payload["tcp_pose"] = {
            "frame_ref": str(tcp_pose.get("frame_ref", "")),
            "xyz_mm": list(tcp_pose.get("xyz_mm", [])),
            "rotation_xyz_deg": list(tcp_pose.get("rotation_xyz_deg", [])),
        }
    block.update(payload)
    save_vision_registry(document, paths)
    return {
        "state": str(block.get("state", "recorded")),
        "updated_at": str(block.get("updated_at", "")),
        "registry_revision": str(document.get("revision", paths.registry_revision)),
    }


def record_marker_for_warehouse(
    warehouse_ref: str,
    paths: VisionCalibrationPaths,
    *,
    arm_snapshot: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    normalized = str(warehouse_ref).strip()
    if not normalized:
        raise ValueError("warehouse_ref 不能为空")
    document = load_vision_registry(paths)
    vision = _vision_registry(document)
    markers = vision.setdefault("markers", {})
    if not isinstance(markers, dict):
        raise ValueError("vision_registry.markers 必须是映射")
    marker_ref = default_marker_ref(normalized)
    frame_ref = device_frame_ref(normalized)
    entry: dict[str, Any] = {
        "marker_ref": marker_ref,
        "label": f"{normalized} 默认 marker",
        "device_frame_ref": frame_ref,
        "recorded_at": utc_now_iso(),
        "evidence_refs": [],
    }
    if arm_snapshot is not None:
        entry["arm_snapshot"] = dict(arm_snapshot)
    markers[normalized] = entry
    save_vision_registry(document, paths)

    calibration = load_calibration(paths)
    frames = calibration.setdefault("frames", {})
    if not isinstance(frames, dict):
        raise ValueError("calibration.frames 必须是映射")
    if frame_ref not in frames:
        frames[frame_ref] = {
            "parent_frame_ref": "arm_base",
            "xyz_m": [0.0, 0.0, 0.0],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        }
    calibration["revision"] = str(calibration.get("revision", paths.calibration_revision))
    save_calibration(calibration, paths)

    return {
        "warehouse_ref": normalized,
        "marker_ref": marker_ref,
        "device_frame_ref": frame_ref,
        "recorded_at": str(entry["recorded_at"]),
        "registry_revision": str(document.get("revision", paths.registry_revision)),
        "calibration_revision": str(calibration.get("revision", paths.calibration_revision)),
    }


def build_card_vision_snapshot(paths: VisionCalibrationPaths) -> dict[str, Any]:
    registry = load_vision_registry(paths)
    calibration = load_calibration(paths)
    vision = _vision_registry(registry)
    camera = vision.get("camera_extrinsic", {})
    tcp = vision.get("tcp_calibration", {})
    bindings_raw = vision.get("point_bindings", {})
    bindings: dict[str, str] = {}
    if isinstance(bindings_raw, dict):
        bindings = {str(key): str(value) for key, value in bindings_raw.items()}
    return {
        "revision": str(registry.get("revision", paths.registry_revision)),
        "calibration_revision": str(
            calibration.get("revision", paths.calibration_revision)
        ),
        "markers": list_markers(paths, registry),
        "point_bindings": bindings,
        "camera_extrinsic_state": str(
            camera.get("state", "pending") if isinstance(camera, dict) else "pending"
        ),
        "tcp_calibration_state": str(
            tcp.get("state", "pending") if isinstance(tcp, dict) else "pending"
        ),
        "capabilities": {
            "camera_extrinsic": True,
            "tcp_calibration": True,
            "marker_record": True,
        },
    }
