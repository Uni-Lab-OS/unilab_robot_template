"""视觉标定登记层（template 通用读写）。"""

from .paths import VisionCalibrationPaths
from .store import (
    bind_point_to_marker,
    build_card_vision_snapshot,
    build_vision_registry_document,
    default_marker_for_warehouse,
    default_marker_ref,
    device_frame_ref,
    list_markers,
    load_calibration,
    load_vision_registry,
    record_camera_extrinsic_stub,
    record_marker_for_warehouse,
    record_tcp_calibration_stub,
    save_calibration,
    save_vision_registry,
    warehouse_from_group_ref,
)

__all__ = [
    "VisionCalibrationPaths",
    "bind_point_to_marker",
    "build_card_vision_snapshot",
    "build_vision_registry_document",
    "default_marker_for_warehouse",
    "default_marker_ref",
    "device_frame_ref",
    "list_markers",
    "load_calibration",
    "load_vision_registry",
    "record_camera_extrinsic_stub",
    "record_marker_for_warehouse",
    "record_tcp_calibration_stub",
    "save_calibration",
    "save_vision_registry",
    "warehouse_from_group_ref",
]
