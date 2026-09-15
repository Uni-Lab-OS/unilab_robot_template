"""Preview 运行时与 L0 模块单元测试。"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest
from unilab_robot_contracts import BackendKind, DeploymentMode, HardwareProfile, InterlockMode
from unilab_robot_runtime import (
    ArmMount,
    PreviewArmDevice,
    RuntimeDependencies,
    build_pick_place_segments,
    create_runtime,
    runtime_requirements,
)
from unilab_robot_runtime.preview.joint_interpolator import validate_duration


@dataclass(frozen=True)
class _FakeKinematics:
    home_deg: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def fk_tcp(self, mount, q_deg, rail_y=0.0):
        del mount, q_deg, rail_y
        return np.eye(4)

    def plate_pose(self, mount, q_deg, rail_y=0.0):
        del mount, q_deg, rail_y
        return np.eye(4)

    def ik_tcp(self, mount, xyz, *, seed=None, yaw_deg=0.0, rail_y=0.0):
        del mount, xyz, seed, yaw_deg, rail_y
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def linear_path(self, mount, start_q, target_xyz, spacing=0.012, rail_y=0.0):
        del mount, target_xyz, spacing, rail_y
        return [list(start_q), list(start_q)]

    def carry_path(self, mount, start_q, target_xyz, rail_y=0.0):
        del mount, target_xyz, rail_y
        return [list(start_q), list(start_q)]


def test_validate_duration_rejects_out_of_range() -> None:
    with pytest.raises(ValueError):
        validate_duration(0.05)
    with pytest.raises(ValueError):
        validate_duration(math.nan)


def test_preview_arm_stop_interrupts_motion() -> None:
    mount = ArmMount("arm_a", (0.0, 0.0, 0.0))
    device = PreviewArmDevice("arm_a", _FakeKinematics(), mount, register=False)
    result: dict[str, object] = {}

    def run() -> None:
        result["value"] = device.set_joint(1, 180.0, 5.0)

    thread = threading.Thread(target=run)
    thread.start()
    time.sleep(0.05)
    device.stop()
    thread.join(timeout=2.0)
    assert result["value"]["status"] == "stopped"  # type: ignore[index]


def test_build_pick_place_segments_returns_six_stages() -> None:
    mount = ArmMount("arm_a", (0.0, 0.0, 0.0))
    segments = build_pick_place_segments(
        _FakeKinematics(),
        mount,
        (0.0, 0.0, 1.0),
        (0.1, 0.1, 1.0),
        approach_source=(0.0, 0.0, 1.2),
        approach_target=(0.1, 0.1, 1.2),
    )
    assert len(segments) == 6
    hover, down, lift, carry, lower, retreat = segments
    assert isinstance(hover, list)
    assert all(len(stage) >= 2 for stage in (down, lift, carry, lower, retreat))


@dataclass(frozen=True)
class _ModuleRef:
    distribution: str
    version: str
    endpoint_ids: frozenset[str]
    python_package: str


@dataclass(frozen=True)
class _PreviewManifest:
    deployment_id: str
    profile: HardwareProfile
    arm: _ModuleRef
    assets: dict[str, object]
    rail: None = None

    def asset_path(self, name: str) -> Path:
        raise ValueError(f"unexpected asset {name}")


def test_runtime_factory_preview_backend(tmp_path: Path) -> None:
    profile = HardwareProfile(
        profile_id="preview-test",
        digest="e" * 64,
        mode=DeploymentMode.SIMULATION,
        backend=BackendKind.PREVIEW,
        endpoint_ids=frozenset({"preview-arm"}),
        interlock_mode=InterlockMode.SIMULATION,
        commissioning_velocity_limit=0.25,
        commissioning_acceleration_limit=0.25,
    )
    manifest = _PreviewManifest(
        "preview-test",
        profile,
        _ModuleRef(
            "unilab-arm-cr7",
            "0.1.0",
            frozenset({"preview-arm"}),
            "unilab_arm_cr7",
        ),
        {},
    )
    mount = ArmMount("preview-arm", (0.0, 0.0, 0.0))
    requirements = runtime_requirements(manifest)
    binding = create_runtime(
        manifest,
        RuntimeDependencies(
            runtime_root=tmp_path,
            preview_kinematics_impl=_FakeKinematics(),
            preview_arm_mount=mount,
        ),
    )
    try:
        assert requirements.preview_kinematics is True
        assert binding.runtime is not None
    finally:
        binding.close()
