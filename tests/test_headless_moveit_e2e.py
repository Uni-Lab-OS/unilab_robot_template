"""Manifest→PointSet→Runtime→MoveGroup 的无 RViz 端到端合同测试。"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pytest
from unilab_robot_contracts import (
    BackendKind,
    CommandState,
    DeploymentMode,
    HardwareProfile,
    InterlockMode,
    MoveTargetCommand,
)
from unilab_robot_runtime import RuntimeDependencies, create_runtime


@dataclass(frozen=True)
class ModuleRef:
    distribution: str
    version: str
    endpoint_ids: frozenset[str]
    python_package: str


@dataclass(frozen=True)
class AssetRef:
    path: Path
    digest: str


@dataclass(frozen=True)
class Manifest:
    deployment_id: str
    profile: HardwareProfile
    arm: ModuleRef
    assets: dict[str, AssetRef]
    rail: None = None

    def asset_path(self, name: str) -> Path:
        return self.assets[name].path


class HeadlessMoveItClient:
    """模拟 move_group/controller，不提供或启动 RViz。"""

    def __init__(self) -> None:
        self.joints = [0.0] * 6
        self.pose = {
            "frame_ref": "arm_base",
            "xyz_m": [0.4, 0.0, 0.3],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        }
        self.max_velocity = 1.0
        self.max_acceleration = 1.0
        self.applied_tools: list[str] = []

    def read_commissioning_state(self) -> dict[str, Any]:
        return {
            "observed_at": time.time(),
            "max_age_s": 1.0,
            "source": "e2e:move_group+controller",
            "online": True,
            "idle": True,
            "active_command_id": None,
            "execution_fenced": False,
            "joint_positions": list(self.joints),
            "tcp_pose": dict(self.pose),
        }

    def move_to_configuration(
        self, *, joint_positions: list[float], joint_names: list[str]
    ) -> None:
        assert len(joint_names) == 6
        self.joints = list(joint_positions)

    def move_to_pose(self, **kwargs: Any) -> None:
        self.pose = {
            "frame_ref": "arm_base",
            "xyz_m": list(kwargs["position"]),
            "orientation_xyzw": list(kwargs["quat_xyzw"]),
        }

    def wait_until_executed(self) -> bool:
        return True

    def cancel_execution(self) -> None:
        return None

    def apply_tool_context(self, context: Any) -> dict[str, Any]:
        self.applied_tools.append(context.digest)
        return {
            "applied": True,
            "tool_context_digest": context.digest,
            "attachment_generation": context.attachment_generation,
        }


class EmptyPayloadPlanningScene:
    """维护点动不触发 pick/place，但 MoveIt 运行时仍必须显式装配该端口。"""

    def snapshot(self) -> list[dict[str, Any]]:
        return []

    def attach_payload(self, **_kwargs: Any) -> dict[str, Any]:
        raise AssertionError("维护点动不应挂载负载")

    def detach_payload(self, **_kwargs: Any) -> dict[str, Any]:
        raise AssertionError("维护点动不应解除负载")


def test_headless_moveit_manifest_can_open_session_and_move_target(
    tmp_path: Path,
) -> None:
    """完整运行时不启动 RViz，也能解析点位并执行维护目标。"""

    points = tmp_path / "points.yaml"
    points.write_text(
        """schema: unilab.robot-point-set/v3
revision: headless-demo@1.0.0
components:
  arm:
    model_ref: package://unilab_arm_cr7/models/model.yaml
    tool_context_ref: tool-demo
installation_calibration:
  revision: headless-calibration@1.0.0
  digest: cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
global:
  arm:
    standby:
      type: joint_positions
      value: [0.0, -0.2, 0.3, 0.0, 0.2, 0.0]
demo:
  device_ref: deck.demo
  targets:
    ready:
      arm:
        type: joint_positions
        value: [0.0, -0.2, 0.3, 0.0, 0.2, 0.0]
""",
        encoding="utf-8",
    )
    tool = tmp_path / "tool.yaml"
    tool.write_text(
        """schema: unilab.tool-context/v1
context_id: tool-demo
attachment_generation: 1
mount_to_tcp:
  xyz_m: [0.0, 0.0, 0.1]
  orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
planning_scene:
  attached_body_id: tool-demo
  parent_link: cr7_link_6
  collision_primitives:
    - primitive_id: demo-envelope
      shape: box
      size_m: [0.1, 0.1, 0.1]
      pose:
        xyz_m: [0.0, 0.0, 0.05]
        orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
  allowed_touch_links: [cr7_link_6]
""",
        encoding="utf-8",
    )
    calibration = tmp_path / "calibration.yaml"
    calibration.write_text(
        """schema: unilab.installation-calibration/v1
revision: headless-calibration@1.0.0
frames:
  device:deck.demo:
    xyz_m: [0.0, 0.0, 0.0]
    orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
""",
        encoding="utf-8",
    )
    endpoint = "moveit:headless-e2e"
    profile = HardwareProfile(
        "headless-e2e",
        "d" * 64,
        DeploymentMode.SIMULATION,
        BackendKind.MOVEIT,
        frozenset({endpoint}),
        InterlockMode.SIMULATION,
        0.25,
        0.25,
    )
    manifest = Manifest(
        "headless-e2e",
        profile,
        ModuleRef(
            "unilab-arm-cr7",
            "0.1.0",
            frozenset({endpoint}),
            "unilab_arm_cr7",
        ),
        {
            "point_set": AssetRef(points, "e" * 64),
            "tool_context": AssetRef(tool, "f" * 64),
            "installation_calibration": AssetRef(calibration, "c" * 64),
        },
    )
    client = HeadlessMoveItClient()
    common_dependencies = {
        "runtime_root": tmp_path / "runtime",
        "moveit_client": client,
        "qualified_joint_names": tuple(
            f"robot_cr7_joint_{index}" for index in range(1, 7)
        ),
    }
    with pytest.raises(ValueError, match="PayloadPlanningScenePort"):
        create_runtime(manifest, RuntimeDependencies(**common_dependencies))
    binding = create_runtime(
        manifest,
        RuntimeDependencies(
            **common_dependencies,
            payload_planning_scene_port=EmptyPayloadPlanningScene(),
        ),
    )
    session = binding.open_maintenance_session("e2e-operator")
    try:
        result = session.execute(
            MoveTargetCommand(
                "headless-command-1",
                profile.digest,
                "boot-e2e",
                1,
                "maintenance-slow",
                0.05,
                0.05,
                "demo.ready",
                "headless-demo@1.0.0",
            )
        )
    finally:
        session.close()
        binding.close()

    assert result.state is CommandState.SUCCEEDED
    assert client.joints == [0.0, -0.2, 0.3, 0.0, 0.2, 0.0]
