"""Preview 运行时与 L0 模块单元测试。"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest
from unilab_robot_contracts import (
    ActionKind,
    BackendKind,
    CommandState,
    DeploymentMode,
    HardwareProfile,
    InterlockMode,
    MotionSegment,
    ObservationState,
    RailStateObservation,
    RobotCommand,
    WorkCellPhaseKind,
)
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


@dataclass(frozen=True)
class _StaticSegmentExecutor:
    calls: list[str]

    def execute_arm_move(self, device, segment, *, stop_event, duration_s) -> None:
        del stop_event, duration_s
        self.calls.append(segment.target_ref)
        device.moveJ(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, duration=0.1)


class _FakeRailPort:
    endpoint_ids = frozenset({"szlab_mixer_rail"})

    def __init__(self) -> None:
        self.moves: list[tuple[str, str]] = []
        self._position = 0.0
        self._target: str | None = None
        self._command_id: str | None = None

    def move(self, command_id: str, target_ref: str) -> None:
        self.moves.append((command_id, target_ref))
        self._command_id = command_id
        self._target = target_ref
        self._position = 0.5 if target_ref == "rail.mid" else 0.0

    def observe(self) -> RailStateObservation:
        known = self._command_id is not None and self._target is not None
        return RailStateObservation(
            ObservationState.KNOWN if known else ObservationState.UNKNOWN,
            time.time(),
            1.0,
            "fake-preview-rail",
            self._position,
            False if known else None,
            True if known else None,
            self._target,
            self._command_id,
        )

    def request_stop(self, command_id: str) -> bool:
        del command_id
        return True


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


def test_runtime_factory_preview_rail_requires_rail_port(tmp_path: Path) -> None:
    profile = HardwareProfile(
        profile_id="preview-rail-test",
        digest="f" * 64,
        mode=DeploymentMode.SIMULATION,
        backend=BackendKind.PREVIEW,
        endpoint_ids=frozenset({"szlab_mixer_robot", "szlab_mixer_rail"}),
        interlock_mode=InterlockMode.SIMULATION,
        commissioning_velocity_limit=0.25,
        commissioning_acceleration_limit=0.25,
    )
    manifest = _PreviewManifest(
        "preview-rail-test",
        profile,
        _ModuleRef(
            "unilab-arm-cr7",
            "0.1.0",
            frozenset({"szlab_mixer_robot"}),
            "unilab_arm_cr7",
        ),
        {},
        rail=_ModuleRef(
            "unilab-rail-linear",
            "0.1.0",
            frozenset({"szlab_mixer_rail"}),
            "unilab_rail_linear",
        ),
    )
    requirements = runtime_requirements(manifest)
    assert requirements.preview_rail_axis_port is True
    with pytest.raises(ValueError, match="RailAxisPort"):
        create_runtime(
            manifest,
            RuntimeDependencies(
                runtime_root=tmp_path,
                preview_kinematics_impl=_FakeKinematics(),
                preview_arm_mount=ArmMount("szlab_mixer_robot", (0.0, 0.0, 0.0), (-1.0, 1.0)),
                preview_segment_executor=_StaticSegmentExecutor([]),
            ),
        )


def test_preview_rail_workcell_executes_rail_then_arm() -> None:
    from unilab_robot_runtime.preview.rail_workcell_runtime import PreviewRailWorkCellRuntime

    rail = _FakeRailPort()
    arm_calls: list[str] = []

    class _ArmStub:
        def validate_before_dispatch(self, command) -> None:
            del command

        def execute(self, command):
            arm_calls.append(command.command_id)
            return type(
                "Result",
                (),
                {
                    "state": CommandState.SUCCEEDED,
                    "message": "ok",
                    "output": {"phases": []},
                },
            )()

        def request_stop(self, command_id, reason):
            del reason
            return type(
                "Result",
                (),
                {"state": CommandState.SUCCEEDED, "message": "stop"},
            )()

    runtime = PreviewRailWorkCellRuntime(arm_runtime=_ArmStub(), rail_port=rail)
    command = RobotCommand(
        command_id="pick-1",
        action=ActionKind.PICK,
        hardware_profile_digest="b" * 64,
        payload_profile="test-profile",
        source_boot_id="boot-1",
        monotonic_sequence=1,
        segments=(
            MotionSegment(
                "approach",
                "S03.pick.approach",
                {"phase_kind": WorkCellPhaseKind.ARM_MOVE.value, "payload_state": "empty"},
            ),
        ),
        metadata={"payload_instance_ref": "00000000-0000-0000-0000-000000000001"},
    )
    result = runtime.execute(command, rail_target_ref="rail.mid")
    assert result.state is CommandState.SUCCEEDED
    assert rail.moves == [("pick-1:rail", "rail.mid")]
    assert arm_calls == ["pick-1:arm"]


def test_preview_arm_execution_backend_runs_segment() -> None:
    from unilab_robot_runtime.preview.arm_execution_backend import PreviewArmExecutionBackend

    mount = ArmMount("arm_a", (0.0, 0.0, 0.0))
    device = PreviewArmDevice("arm_a", _FakeKinematics(), mount, register=False)
    calls: list[str] = []
    executor = _StaticSegmentExecutor(calls)
    backend = PreviewArmExecutionBackend(
        device=device,
        segment_executor=executor,
        endpoint_ids=frozenset({"arm_a"}),
    )
    command = RobotCommand(
        command_id="cmd-1",
        action=ActionKind.PICK,
        hardware_profile_digest="b" * 64,
        payload_profile="test-profile",
        source_boot_id="boot-1",
        monotonic_sequence=1,
        segments=(
            MotionSegment(
                "approach",
                "S03.pick.approach",
                {"phase_kind": WorkCellPhaseKind.ARM_MOVE.value, "payload_state": "empty"},
            ),
        ),
        metadata={"payload_instance_ref": "00000000-0000-0000-0000-000000000001"},
    )
    result = backend.execute(command)
    assert result.state is CommandState.SUCCEEDED
    assert calls == ["S03.pick.approach"]
