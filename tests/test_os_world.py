"""机器人领域库直接实现 OS 单世界协议的行为测试。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import pytest
from unilab_robot_contracts import ActionKind, CommandResult, CommandState
from unilab_robot_runtime.os_world import RobotWorld, WorldIdentity

IDENTITY = WorldIdentity(world_id="world-sim", session_id="session-1", epoch=7)


@dataclass
class _Binding:
    """记录派发命令并返回预设结果的等价 execute(command) 可调用对象。"""

    state: CommandState = CommandState.SUCCEEDED
    error: BaseException | None = None
    calls: list[Any] = field(default_factory=list)

    def execute(self, command: Any) -> CommandResult:
        self.calls.append(command)
        if self.error is not None:
            raise self.error
        return CommandResult(command.command_id, self.state, "test")


def _command_mapping() -> dict[str, Any]:
    """返回命令名到显式 RobotCommand 字段的映射，不做隐式推断。"""

    return {
        "pick": {
            "action": "pick",
            "payload_profile": "beaker_500ml@v1",
            "source_boot_id": "boot-1",
            "segments": [
                {"segment_id": "approach", "target_ref": "S04.pick.approach"},
                {"segment_id": "grip", "target_ref": "end_effector.grip"},
            ],
        }
    }


def _world(binding: _Binding | None = None) -> RobotWorld:
    """按显式身份与命令映射构造世界，默认注入成功绑定。"""

    return RobotWorld(
        binding if binding is not None else _Binding(),
        IDENTITY,
        hardware_profile_digest="profile-digest-1",
        commands=_command_mapping(),
    )


def test_command_maps_name_and_parameters_to_robot_command_once() -> None:
    """命令名/显式参数映射为 RobotCommand，同一请求只派发一次。"""

    binding = _Binding()
    world = _world(binding)
    receipt = world.command("pick", command_id="cmd-42", attempt=3)
    assert len(binding.calls) == 1
    command = binding.calls[0]
    assert command.command_id == "cmd-42"
    assert command.action is ActionKind.PICK
    assert command.hardware_profile_digest == "profile-digest-1"
    assert command.payload_profile == "beaker_500ml@v1"
    assert command.source_boot_id == "boot-1"
    assert command.monotonic_sequence == 1
    assert [segment.target_ref for segment in command.segments] == [
        "S04.pick.approach",
        "end_effector.grip",
    ]
    assert receipt["command_id"] == "cmd-42"
    assert receipt["state"] == "succeeded"
    assert receipt["settled"] is True
    assert receipt["epoch"] == 7
    assert receipt["tick"] == 0
    assert receipt["attempt"] == 3
    assert world.command("pick", command_id="cmd-42", attempt=3) == receipt
    with pytest.raises(ValueError, match="world_command_busy"):
        world.command("pick", command_id="cmd-43")
    assert len(binding.calls) == 1


def test_scan_advances_only_after_command_settles() -> None:
    """未结算命令不得推进 tick；结算后的命令才允许推进。"""

    running = _world(_Binding(CommandState.RUNNING))
    receipt = running.command("pick")
    assert receipt["settled"] is False
    with pytest.raises(RuntimeError, match="world_command_unsettled"):
        running.scan()
    assert running.tick == 0
    assert running.observe()["tick"] == 0

    settled = _world(_Binding(CommandState.SUCCEEDED))
    settled.command("pick")
    assert settled.scan()["tick"] == 1
    assert settled.tick == 1
    assert settled.scan()["tick"] == 2


def test_unknown_or_failed_command_locks_world_without_replay() -> None:
    """未知命令、绑定异常和 execution_unknown 都永久锁定且不重放。"""

    unknown = _world()
    with pytest.raises(ValueError, match="world_command_unknown"):
        unknown.command("fly")
    with pytest.raises(RuntimeError, match="world_result_unknown"):
        unknown.command("pick")
    with pytest.raises(RuntimeError, match="world_result_unknown"):
        unknown.scan()
    with pytest.raises(RuntimeError, match="world_result_unknown"):
        unknown.observe()

    failed = _Binding(error=RuntimeError("dispatch_failed"))
    failed_world = _world(failed)
    with pytest.raises(RuntimeError, match="dispatch_failed"):
        failed_world.command("pick")
    with pytest.raises(RuntimeError, match="world_result_unknown"):
        failed_world.scan()
    assert len(failed.calls) == 1

    uncertain = _world(_Binding(CommandState.EXECUTION_UNKNOWN))
    receipt = uncertain.command("pick")
    assert receipt["state"] == "execution_unknown"
    assert receipt["settled"] is False
    with pytest.raises(RuntimeError, match="world_result_unknown"):
        uncertain.scan()
    with pytest.raises(RuntimeError, match="world_result_unknown"):
        uncertain.command("pick")


def test_build_device_runtime_returns_os_device_runtime_and_world_runs() -> None:
    """设备包入口返回 OS DeviceRuntime，并被 OS 执行循环直接调用。"""

    from unilabos.simulation.execution import DeviceWorldExecutor
    from unilabos.simulation.execution_service import DeviceRuntime

    from unilab_robot_runtime.os_world import build_device_runtime

    binding = _Binding()
    recorded: list[Any] = []
    config = {
        "world": {"world_id": "world-sim", "session_id": "session-os", "epoch": 3},
        "profile": {"digest": "profile-digest-os"},
        "commands": _command_mapping(),
        "binding": binding,
        "bindings": {"szlab_mixer_robot": {"uuid": "material-1", "actions": {}}},
    }
    runtime = build_device_runtime(config, object(), recorded.append)
    assert isinstance(runtime, DeviceRuntime)
    assert isinstance(runtime.world, RobotWorld)
    assert runtime.bindings == config["bindings"]

    request = {
        "identity": {"node_job_uuid": "job-1", "task_uuid": "task-1"},
        "operation": "pick",
        "parameters": {},
    }
    settled: list[Any] = []

    def settle(_request: Any, proof: Any) -> dict[str, str]:
        settled.append(proof)
        return {"receipt_id": "receipt-1"}

    executor = DeviceWorldExecutor(
        runtime.world,
        runtime.model,
        settle=settle,
        authorize=lambda _request: None,
        record=recorded.append,
    )
    result = executor.execute(request)
    assert result["success"] is True
    assert result["inventory_receipt"] == {"receipt_id": "receipt-1"}
    assert len(binding.calls) == 1
    assert settled[0]["identity"] == request["identity"]
    assert settled[0]["command_id"] == binding.calls[0].command_id
    assert runtime.world.tick == 2

    fake_model = object()
    injected = build_device_runtime({**config, "model": fake_model}, object(), lambda _: None)
    assert injected.model is fake_model

    without_binding = {k: v for k, v in config.items() if k != "binding"}
    with pytest.raises(ValueError, match="binding"):
        build_device_runtime(without_binding, {}, lambda _: None)


def test_scan_rejects_foreign_observation_and_locks_world() -> None:
    """注入的扫描观察身份/tick 不符时锁定；一致时才成为已确认观察。"""

    def foreign(tick: int) -> dict[str, Any]:
        return {
            "world_id": "other-world",
            "session_id": "session-1",
            "epoch": 7,
            "tick": tick,
            "observation": {},
        }

    world = RobotWorld(
        _Binding(),
        IDENTITY,
        hardware_profile_digest="profile-digest-1",
        commands=_command_mapping(),
        scan_advance=foreign,
    )
    with pytest.raises(ValueError, match="world_observation_identity_mismatch"):
        world.scan()
    assert world.faulted is True
    with pytest.raises(RuntimeError, match="world_result_unknown"):
        world.observe()

    def confirmed(tick: int) -> dict[str, Any]:
        return {
            **IDENTITY.as_mapping(),
            "tick": tick,
            "observation": {"joint_positions": [0.1]},
        }

    ok = RobotWorld(
        _Binding(),
        IDENTITY,
        hardware_profile_digest="profile-digest-1",
        commands=_command_mapping(),
        scan_advance=confirmed,
    )
    observation = ok.scan()
    assert observation["tick"] == 1
    assert observation["observation"] == {"joint_positions": [0.1]}
    assert ok.observe() == observation

    def broken(_tick: int) -> dict[str, Any]:
        raise RuntimeError("advance_failed")

    broken_world = RobotWorld(
        _Binding(),
        IDENTITY,
        hardware_profile_digest="profile-digest-1",
        commands=_command_mapping(),
        scan_advance=broken,
    )
    with pytest.raises(RuntimeError, match="advance_failed"):
        broken_world.scan()
    with pytest.raises(RuntimeError, match="world_result_unknown"):
        broken_world.scan()


def test_observe_reports_identity_and_confirmed_tick() -> None:
    """观察包络只来自世界身份与已确认 tick，不伪造物理量。"""

    world = _world()
    assert world.observe() == {
        "world_id": "world-sim",
        "session_id": "session-1",
        "epoch": 7,
        "tick": 0,
        "sim_time": 0.0,
        "observation": {},
    }
    scan = world.scan()
    assert scan["tick"] == 1
    assert world.tick == 1
    assert world.observe() == scan
