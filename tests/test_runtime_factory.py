"""通用运行时选择和多后端装配合同测试。"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from unilab_robot_contracts import (
    ActionKind,
    BackendKind,
    CommandState,
    DeploymentMode,
    HardwareProfile,
    InterlockMode,
    MotionSegment,
    ObservationState,
    RobotCommand,
    SafetyInterlockObservation,
)
from unilab_robot_runtime import (
    RuntimeDependencies,
    create_runtime,
    runtime_requirements,
)


@dataclass(frozen=True)
class ModuleRef:
    """测试使用的 exact module 引用。"""

    distribution: str
    version: str
    endpoint_ids: frozenset[str]
    python_package: str


@dataclass(frozen=True)
class AssetRef:
    """测试 manifest 的 exact 资产引用。"""

    path: Path
    digest: str


@dataclass(frozen=True)
class Manifest:
    """TCP/SDK standalone 运行时所需的最小清单。"""

    deployment_id: str
    profile: HardwareProfile
    arm: ModuleRef
    assets: dict[str, AssetRef]
    rail: None = None

    def asset_path(self, name: str) -> Path:
        """返回 TCP/SDK 与 MoveIt 共用的 PointSet v3 资产。"""

        return self.assets[name].path


class FakeSdkPort:
    """返回精确完成见证的受限厂家 SDK 端口。"""

    def __init__(self) -> None:
        """创建空目标调用记录。"""

        self.targets: list[str] = []

    def execute_target(
        self,
        target_ref: str,
        parameters: dict[str, Any],
        command_id: str,
    ) -> dict[str, Any]:
        """记录白名单目标并返回绑定命令的完成见证。"""

        del parameters
        self.targets.append(target_ref)
        return {"command_id": command_id, "state": "succeeded", "completed": True}

    def query_command(self, command_id: str) -> dict[str, Any]:
        """返回绑定同一命令的完成见证。"""

        return {"command_id": command_id, "state": "succeeded", "completed": True}

    def request_stop(self, command_id: str, reason: str) -> bool:
        """确认测试命令停止；原因只用于接口一致。"""

        del command_id, reason
        return True


def _manifest(tmp_path: Path, endpoint: str = "tcp:cr7:test") -> Manifest:
    """创建使用 PointSet v3 的 TCP/SDK standalone 清单。"""

    profile = HardwareProfile(
        profile_id="tcp-sdk-test",
        digest="a" * 64,
        mode=DeploymentMode.SIMULATION,
        backend=BackendKind.TCP_SDK,
        endpoint_ids=frozenset({endpoint}),
        interlock_mode=InterlockMode.SIMULATION,
        commissioning_velocity_limit=0.25,
        commissioning_acceleration_limit=0.25,
    )
    point_set = tmp_path / "points.yaml"
    point_set.write_text(
        """schema: unilab.robot-point-set/v3
revision: sdk-points@1.0.0
components:
  arm:
    model_ref: package://unilab_arm_cr7/models/model.yaml
    tool_context_ref: sdk-tool
installation_calibration:
  revision: sdk-installation@1.0.0
  digest: cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
global:
  arm:
    standby: {type: joint_positions, value: [0, 0, 0, 0, 0, 0]}
position1:
  device_ref: deck.position1
  targets:
    approach:
      arm: {type: joint_positions, value: [0, 0, 0, 0, 0, 0]}
""",
        encoding="utf-8",
    )
    tool = tmp_path / "tool.yaml"
    tool.write_text(
        """schema: unilab.tool-context/v1
context_id: sdk-tool
attachment_generation: 1
mount_to_tcp:
  xyz_m: [0, 0, 0]
  orientation_xyzw: [0, 0, 0, 1]
""",
        encoding="utf-8",
    )
    calibration = tmp_path / "calibration.yaml"
    calibration.write_text(
        """schema: unilab.installation-calibration/v1
revision: sdk-installation@1.0.0
frames:
  device:deck.position1:
    xyz_m: [0, 0, 0]
    orientation_xyzw: [0, 0, 0, 1]
""",
        encoding="utf-8",
    )
    return Manifest(
        "tcp-sdk-test",
        profile,
        ModuleRef(
            "unilab-arm-cr7",
            "0.1.0",
            frozenset({endpoint}),
            "unilab_arm_cr7",
        ),
        {
            "point_set": AssetRef(point_set, "b" * 64),
            "tool_context": AssetRef(tool, "d" * 64),
            "installation_calibration": AssetRef(calibration, "c" * 64),
        },
    )


def _safety() -> SafetyInterlockObservation:
    """返回显式仿真许可，不冒充硬件互锁。"""

    return SafetyInterlockObservation(
        ObservationState.KNOWN,
        time.time(),
        1.0,
        "test-simulation",
        True,
        False,
        False,
        True,
        True,
    )


def test_runtime_selects_tcp_sdk_without_domain_branch(tmp_path: Path) -> None:
    """HardwareProfile 的 TCP/SDK 判断只发生在共享运行时。"""

    manifest = _manifest(tmp_path)
    requirements = runtime_requirements(manifest)
    sdk = FakeSdkPort()
    binding = create_runtime(
        manifest,
        RuntimeDependencies(
            runtime_root=tmp_path,
            robot_sdk_port=sdk,
            safety_observation=_safety,
        ),
    )
    command = RobotCommand(
        command_id="tcp-command-1",
        action=ActionKind.PICK,
        hardware_profile_digest=manifest.profile.digest,
        payload_profile="beaker",
        source_boot_id="test-boot",
        monotonic_sequence=1,
        segments=(
            MotionSegment(
                "approach",
                "position1.approach",
                {"phase_kind": "arm_move", "payload_state": "empty"},
            ),
            MotionSegment(
                "pick",
                "end_effector.grip",
                {"phase_kind": "end_effector", "payload_state": "loaded"},
            ),
            MotionSegment(
                "observe-payload",
                "end_effector.payload.loaded",
                {"phase_kind": "observe", "payload_state": "loaded"},
            ),
            MotionSegment(
                "retract",
                "position1.approach",
                {"phase_kind": "arm_move", "payload_state": "loaded"},
            ),
        ),
    )

    try:
        result = binding.execute(command)
    finally:
        binding.close()

    assert requirements.robot_sdk_port is True
    assert requirements.variable_port is False
    assert requirements.moveit_client is False
    assert result.state is CommandState.SUCCEEDED
    assert sdk.targets == ["position1.approach", "position1.approach"]


def test_workspace_has_no_legacy_second_execution_surface() -> None:
    """旧总包、execute_skill profile 和 v1 点位不得重新成为发布权威。"""

    root = Path(__file__).resolve().parents[1]
    assert not tuple((root / "src/unilab_robot").rglob("*.py"))
    assert not (root / "profile/device.yaml").exists()
    assert not (root / "config/point_set.example.yaml").exists()
    assert not (root / "ros2_ws/src/unilab_robot_msgs/action/ExecuteSkill.action").exists()
