"""Vision 登记读写（template 通用层）。"""

from __future__ import annotations

from pathlib import Path

import pytest

from unilab_robot_runtime.vision_calibration import (
    VisionCalibrationPaths,
    bind_point_to_marker,
    build_card_vision_snapshot,
    default_marker_ref,
    device_frame_ref,
    load_vision_registry,
    record_camera_extrinsic_stub,
    record_marker_for_warehouse,
    record_tcp_calibration_stub,
    warehouse_from_group_ref,
)


def _paths(
    *,
    calibration: Path,
    registry: Path,
) -> VisionCalibrationPaths:
    return VisionCalibrationPaths(
        calibration_path=calibration,
        registry_path=registry,
        registry_revision="test-vision-registry@1.0.0",
        calibration_revision="test-calibration@1.0.0",
        tool_context_ref="package://test/tool_context.yaml",
    )


@pytest.fixture
def temp_assets(tmp_path: Path) -> VisionCalibrationPaths:
    calibration = tmp_path / "calibration.v1.yaml"
    registry = tmp_path / "vision-registry.v1.yaml"
    calibration.write_text(
        "schema: unilab.installation-calibration/v1\nrevision: test\nframes: {}\n",
        encoding="utf-8",
    )
    return _paths(calibration=calibration, registry=registry)


def test_default_marker_ref_and_device_frame() -> None:
    assert default_marker_ref("s07_process_warehouse") == "s07_process_warehouse/default"
    assert device_frame_ref("s07_process_warehouse") == "device:s07_process_warehouse"
    assert warehouse_from_group_ref("s07_process_warehouse") == "s07_process_warehouse"


def test_record_marker_writes_registry_and_calibration_frame(temp_assets: VisionCalibrationPaths) -> None:
    result = record_marker_for_warehouse(
        "s07_process_warehouse",
        temp_assets,
        arm_snapshot={"joint_positions_si": [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]},
    )
    assert result["marker_ref"] == "s07_process_warehouse/default"
    registry = load_vision_registry(temp_assets)
    markers = registry["vision_registry"]["markers"]
    assert "s07_process_warehouse" in markers
    calibration = temp_assets.calibration_path.read_text(encoding="utf-8")
    assert "device:s07_process_warehouse" in calibration


def test_bind_point_to_marker_persists_binding(temp_assets: VisionCalibrationPaths) -> None:
    result = bind_point_to_marker(
        "s07_process_warehouse.S0722.interaction_seed",
        "s07_process_warehouse/default",
        temp_assets,
    )
    assert result["target_ref"].endswith("interaction_seed")
    snapshot = build_card_vision_snapshot(temp_assets)
    assert (
        snapshot["point_bindings"]["s07_process_warehouse.S0722.interaction_seed"]
        == "s07_process_warehouse/default"
    )


def test_record_camera_extrinsic_stub_updates_registry(temp_assets: VisionCalibrationPaths) -> None:
    record_camera_extrinsic_stub(temp_assets)
    registry = load_vision_registry(temp_assets)
    assert registry["vision_registry"]["camera_extrinsic"]["state"] == "recorded"


def test_record_tcp_calibration_stub_updates_registry(temp_assets: VisionCalibrationPaths) -> None:
    record_tcp_calibration_stub(temp_assets)
    registry = load_vision_registry(temp_assets)
    assert registry["vision_registry"]["tcp_calibration"]["state"] == "recorded"
