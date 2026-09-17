"""Vision 动作基于 ExecutorObservation。"""

from __future__ import annotations

from pathlib import Path

from unilab_robot_runtime.device_card.observation_types import ExecutorObservation
from unilab_robot_runtime.device_card.vision_actions import (
    calibrate_camera_extrinsic_from_observation,
)
from unilab_robot_runtime.vision_calibration.paths import VisionCalibrationPaths


def test_calibrate_camera_extrinsic_from_observation(tmp_path: Path) -> None:
    registry = tmp_path / "registry.yaml"
    registry.write_text(
        "schema: unilab.robot-qualifications/v1\nrevision: demo@1.0.0\n"
        "vision_registry:\n  camera_extrinsic:\n    state: pending\n"
        "  tcp_calibration:\n    state: pending\n  markers: {}\n  point_bindings: {}\n",
        encoding="utf-8",
    )
    paths = VisionCalibrationPaths(
        calibration_path=tmp_path / "cal.yaml",
        registry_path=registry,
        registry_revision="demo@1.0.0",
        calibration_revision="demo@1.0.0",
        tool_context_ref="demo-tool@1.0.0",
    )
    obs: ExecutorObservation = {"online": True, "idle": True, "joint_positions_si": [0.0] * 6}
    result = calibrate_camera_extrinsic_from_observation(obs, paths, confirm=True)
    assert result["state"] == "recorded"
