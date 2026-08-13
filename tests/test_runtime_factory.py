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
class Manifest:
    """TCP/SDK standalone 运行时所需的最小清单。"""

    deployment_id: str
    profile: HardwareProfile
    arm: ModuleRef
    rail: None = None

    def asset_path(self, name: str) -> Path:
        """TCP/SDK 不读取部署文件；意外读取即使测试失败。"""

        raise AssertionError(f"TCP/SDK 不应读取资产: {name}")


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


def _manifest(endpoint: str = "tcp:cr7:test") -> Manifest:
    """创建不依赖领域包的 TCP/SDK standalone 清单。"""

    profile = HardwareProfile(
        profile_id="tcp-sdk-test",
        digest="a" * 64,
        mode=DeploymentMode.SIMULATION,
        backend=BackendKind.TCP_SDK,
        endpoint_ids=frozenset({endpoint}),
        interlock_mode=InterlockMode.SIMULATION,
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

    manifest = _manifest()
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
        segments=(MotionSegment("approach", "position1.approach"),),
    )

    try:
        result = binding.execute(command)
    finally:
        binding.close()

    assert requirements.robot_sdk_port is True
    assert requirements.variable_port is False
    assert requirements.moveit_client is False
    assert result.state is CommandState.SUCCEEDED
    assert sdk.targets == ["position1.approach"]


def test_workspace_has_no_legacy_second_execution_surface() -> None:
    """旧总包、execute_skill profile 和 v1 点位不得重新成为发布权威。"""

    root = Path(__file__).resolve().parents[1]
    assert not tuple((root / "src/unilab_robot").rglob("*.py"))
    assert not (root / "profile/device.yaml").exists()
    assert not (root / "config/point_set.example.yaml").exists()
    assert not (root / "ros2_ws/src/unilab_robot_msgs/action/ExecuteSkill.action").exists()
