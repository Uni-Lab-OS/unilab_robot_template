"""机器人领域库直接实现 OS 单世界协议；不设适配层，也不复制 OS 执行循环。

``RobotWorld`` 只承担三件事：把 OS 命令名映射成 ``RobotCommand`` 交给领域绑定、
按已确认 tick 返回观察、在结果未知后永久撤销推进许可。库存结算、任务身份与
完成判据仍由 OS 的 ``DeviceWorldExecutor``/``NativeJobGate`` 拥有。
"""

from __future__ import annotations

import json
from collections.abc import Callable, Mapping
from copy import deepcopy
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from unilab_robot_contracts import (
    ActionKind,
    CommandResult,
    CommandState,
    MotionSegment,
    RobotCommand,
)

if TYPE_CHECKING:
    from unilabos.simulation.execution_service import DeviceRuntime

DEFAULT_STEP_SECONDS = 0.05


def _require_text(value: Any, field_name: str) -> str:
    """要求非空字符串，避免用隐式转换掩盖身份错误。"""

    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field_name} 必须是非空字符串")
    return value


@dataclass(frozen=True)
class WorldIdentity:
    """OS 单世界的代际身份；与 PLC-Sim ``StepToken`` 前三段一致。"""

    world_id: str
    session_id: str
    epoch: int = 0

    def __post_init__(self) -> None:
        """拒绝空世界/会话名和非整数代际。"""

        _require_text(self.world_id, "world_id")
        _require_text(self.session_id, "session_id")
        if type(self.epoch) is not int or self.epoch < 0:
            raise ValueError("epoch 必须是非负整数")

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> WorldIdentity:
        """只读取显式的 world_id/session_id/epoch 字段。"""

        if not isinstance(data, Mapping):
            raise TypeError("世界身份必须是对象")
        missing = [
            key for key in ("world_id", "session_id", "epoch") if key not in data
        ]
        if missing:
            raise ValueError(f"世界身份缺少字段: {missing}")
        return cls(data["world_id"], data["session_id"], data["epoch"])

    def as_mapping(self) -> dict[str, Any]:
        """返回可写入 OS 观察包络的稳定身份字段。"""

        return {
            "world_id": self.world_id,
            "session_id": self.session_id,
            "epoch": self.epoch,
        }


def _noop_record(_event: Mapping[str, Any]) -> None:
    """未注入记录器时丢弃世界事件，不伪造审计。"""


def _motion_segments(values: Any) -> tuple[MotionSegment, ...]:
    """把显式段描述转换为已验证的 ``MotionSegment``，不做占位补全。"""

    if not isinstance(values, (list, tuple)) or not values:
        raise ValueError("命令映射至少需要一个已解析运动段")
    segments: list[MotionSegment] = []
    for value in values:
        if isinstance(value, MotionSegment):
            segments.append(value)
            continue
        if not isinstance(value, Mapping):
            raise TypeError("运动段必须是 MotionSegment 或对象")
        unknown = set(value) - {"segment_id", "target_ref", "parameters"}
        if unknown:
            raise ValueError(f"运动段包含未知字段: {sorted(unknown)}")
        segment_id, target_ref = value.get("segment_id"), value.get("target_ref")
        if not isinstance(segment_id, str) or not isinstance(target_ref, str):
            raise TypeError("运动段必须有字符串 segment_id 与 target_ref")
        segments.append(
            MotionSegment(
                segment_id,
                target_ref,
                dict(value.get("parameters") or {}),
            )
        )
    return tuple(segments)


@dataclass(frozen=True)
class CommandSpec:
    """OS 命令名到 ``RobotCommand`` 模板的显式映射，不做隐式推断。"""

    action: ActionKind
    payload_profile: str
    source_boot_id: str
    segments: tuple[MotionSegment, ...]

    def __post_init__(self) -> None:
        """校验动作、负载、启动代次与运动段。"""

        if not isinstance(self.action, ActionKind):
            raise TypeError("action 必须是 ActionKind")
        _require_text(self.payload_profile, "payload_profile")
        _require_text(self.source_boot_id, "source_boot_id")
        if not self.segments or any(
            not isinstance(segment, MotionSegment) for segment in self.segments
        ):
            raise ValueError("segments 必须是非空 MotionSegment 序列")

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> CommandSpec:
        """只读取 action/payload_profile/source_boot_id/segments 四个显式字段。"""

        if not isinstance(data, Mapping):
            raise TypeError("命令映射必须是对象")
        unknown = set(data) - {
            "action",
            "payload_profile",
            "source_boot_id",
            "segments",
        }
        if unknown:
            raise ValueError(f"命令映射包含未知字段: {sorted(unknown)}")
        missing = [
            key
            for key in ("action", "payload_profile", "source_boot_id")
            if key not in data
        ]
        if missing:
            raise ValueError(f"命令映射缺少字段: {missing}")
        return cls(
            action=ActionKind(data["action"]),
            payload_profile=data["payload_profile"],
            source_boot_id=data["source_boot_id"],
            segments=_motion_segments(data.get("segments")),
        )


class RobotWorld:
    """按 OS 单世界协议（observe/command/scan/invalidate）暴露机器人领域执行。

    参数：注入的机器人绑定、世界身份、显式命令映射与可选扫描推进函数。
    返回：无。异常：身份非法、扫描/观察与身份或 tick 不一致时关闭失败。
    """

    def __init__(
        self,
        binding: Any,
        identity: WorldIdentity,
        *,
        hardware_profile_digest: str,
        commands: Mapping[str, Any],
        record: Callable[[Mapping[str, Any]], None] | None = None,
        scan_advance: Callable[[int], Mapping[str, Any]] | None = None,
        step_seconds: float = DEFAULT_STEP_SECONDS,
    ) -> None:
        if not isinstance(identity, WorldIdentity):
            raise TypeError("identity 必须是 WorldIdentity")
        if not callable(getattr(binding, "execute", None)):
            raise TypeError("binding 必须提供 execute(command) -> CommandResult")
        _require_text(hardware_profile_digest, "hardware_profile_digest")
        if not isinstance(commands, Mapping) or not commands:
            raise ValueError("commands 必须是非空命令映射")
        if type(step_seconds) not in (int, float) or step_seconds <= 0:
            raise ValueError("step_seconds 必须为正数")
        self._binding = binding
        self._identity = identity
        self._hardware_profile_digest = hardware_profile_digest
        self._commands = {
            str(name): (
                spec if isinstance(spec, CommandSpec) else CommandSpec.from_mapping(spec)
            )
            for name, spec in commands.items()
        }
        self._record_callback = record or _noop_record
        self._scan_advance = scan_advance
        self._step_seconds = float(step_seconds)
        self.tick = 0
        self._faulted = False
        self._observation: dict[str, Any] | None = None
        self._sequence = 0
        self._issued_receipt: dict[str, Any] | None = None
        self._issued_fingerprint: str | None = None
        self._settled = False

    @property
    def identity(self) -> dict[str, Any]:
        """返回世界身份副本，供 OS/ScheduledWorldClient 比对。"""

        return self._identity.as_mapping()

    @property
    def faulted(self) -> bool:
        """返回世界是否已被永久撤销推进许可。"""

        return self._faulted

    def _require_not_faulted(self) -> None:
        """结果未知后拒绝任何观察、命令或推进。"""

        if self._faulted:
            raise RuntimeError("world_result_unknown")

    def _observation_for(self, tick: int) -> dict[str, Any]:
        """按已确认 tick 构造观察包络；默认不带任何物理载荷。"""

        return {
            **self._identity.as_mapping(),
            "tick": tick,
            "sim_time": round(tick * self._step_seconds, 9),
            "observation": {},
        }

    def observe(self) -> dict[str, Any]:
        """返回与身份和当前 tick 一致的观察；不一致时锁定并抛错。"""

        self._require_not_faulted()
        observation = self._observation
        if observation is None:
            observation = self._observation_for(self.tick)
            self._observation = observation
        self._confirm_observation(observation, self.tick)
        return deepcopy(observation)

    def _confirm_observation(self, observation: Mapping[str, Any], tick: int) -> None:
        """核对观察包络的身份与 tick，任何偏差都永久锁定世界。"""

        expected = {**self._identity.as_mapping(), "tick": tick}
        if not isinstance(observation, Mapping) or any(
            observation.get(key) != value for key, value in expected.items()
        ):
            self.invalidate(reason="observation_identity_mismatch")
            raise ValueError("world_observation_identity_mismatch")

    def command(self, name: str, **parameters: Any) -> dict[str, Any]:
        """把 OS 命令名与显式参数映射为 ``RobotCommand`` 并派发一次。

        参数：命令名与保留字段 ``command_id``/``attempt``/``metadata``/``segments``。
        返回：可写入 OS 日志的结算回执，不携带伪造的物理观测。
        异常：未知命令、非法参数或绑定失败时撤销许可并向上抛出。
        """

        self._require_not_faulted()
        spec = self._commands.get(str(name))
        if spec is None:
            self.invalidate(reason="command_unknown")
            raise ValueError(f"world_command_unknown: {name}")
        options = dict(parameters)
        command_id = options.pop("command_id", None)
        attempt = options.pop("attempt", None)
        metadata = dict(options.pop("metadata", None) or {})
        segments = (
            _motion_segments(options.pop("segments"))
            if "segments" in options
            else spec.segments
        )
        if options:
            raise ValueError(
                f"world_command_parameter_unsupported: {sorted(options)}"
            )
        sequence = self._sequence + 1
        if command_id is not None:
            explicit_id = _require_text(command_id, "command_id")
        else:
            explicit_id = ""
        if attempt is not None and (type(attempt) is not int or attempt < 1):
            raise ValueError("attempt 必须是正整数")
        request_key = json.dumps(
            {
                "name": str(name),
                "command_id": explicit_id or None,
                "attempt": attempt,
                "metadata": metadata,
                "segments": [
                    {
                        "segment_id": segment.segment_id,
                        "target_ref": segment.target_ref,
                        "parameters": dict(segment.parameters),
                    }
                    for segment in segments
                ],
            },
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        )
        if self._issued_receipt is not None:
            if request_key != self._issued_fingerprint:
                raise ValueError("world_command_busy")
            return deepcopy(self._issued_receipt)
        command = RobotCommand(
            command_id=(
                explicit_id
                or (
                    f"{self._identity.world_id}:{self._identity.session_id}:"
                    f"{self._identity.epoch}:{self.tick}:{sequence}"
                )
            ),
            action=spec.action,
            hardware_profile_digest=self._hardware_profile_digest,
            payload_profile=spec.payload_profile,
            source_boot_id=spec.source_boot_id,
            monotonic_sequence=sequence,
            segments=segments,
            metadata={"os_command_name": str(name), **metadata},
        )
        self._sequence = sequence
        try:
            result = self._binding.execute(command)
        except BaseException:
            self.invalidate(reason="command_execution_failed")
            raise
        if not isinstance(result, CommandResult):
            self.invalidate(reason="command_result_invalid")
            raise TypeError("binding.execute 必须返回 CommandResult")
        if result.command_id != command.command_id:
            self.invalidate(reason="command_result_identity_mismatch")
            raise ValueError("world_command_result_identity_mismatch")
        receipt = {
            "command_id": command.command_id,
            "state": result.state.value,
            "settled": result.state.terminal,
            "epoch": self._identity.epoch,
            "tick": self.tick,
            "sequence": command.monotonic_sequence,
            "attempt": attempt,
            "message": result.message,
            "output": deepcopy(dict(result.output)),
        }
        self._issued_receipt, self._issued_fingerprint = receipt, request_key
        self._settled = result.state.terminal
        if result.state is CommandState.EXECUTION_UNKNOWN:
            self.invalidate(reason="command_execution_unknown")
        return deepcopy(receipt)

    def scan(self) -> dict[str, Any]:
        """推进一个已确认 tick 并返回观察；默认只递增 tick，不做物理。"""

        self._require_not_faulted()
        if self._issued_receipt is not None and not self._settled:
            raise RuntimeError("world_command_unsettled")
        next_tick = self.tick + 1
        try:
            observation = (
                self._observation_for(next_tick)
                if self._scan_advance is None
                else dict(self._scan_advance(next_tick))
            )
        except BaseException:
            self.invalidate(reason="scan_advance_failed")
            raise
        self._confirm_observation(observation, next_tick)
        self.tick = next_tick
        self._observation = deepcopy(observation)
        self._issued_receipt = None
        self._issued_fingerprint = None
        self._settled = False
        return deepcopy(observation)

    def invalidate(self, *, reason: str = "invalidate") -> None:
        """永久撤销推进许可；已发出的动作结果保持未知，不重放。"""

        if self._faulted:
            return
        self._faulted = True
        self._record_callback(
            {
                "event": "world.invalidated",
                "reason": reason,
                **self._identity.as_mapping(),
                "tick": self.tick,
            }
        )


class RobotWorldModel:
    """OS ``DeviceWorldExecutor`` 的行为模型：把操作名交给 ``RobotWorld``。

    只把绑定返回的 ``CommandResult`` 投影成完成证据；库存结算仍由 OS 的
    ``settle`` 完成，因此本模型不产生 ``pending_effect``，也不伪造物理观测。
    """

    def __init__(self, world: RobotWorld) -> None:
        if not isinstance(world, RobotWorld):
            raise TypeError("RobotWorldModel 必须包裹 RobotWorld")
        self.world = world
        self._request_identity: dict[str, Any] | None = None
        self._receipt: dict[str, Any] | None = None
        self._state: dict[str, Any] | None = None

    @property
    def world_id(self) -> str:
        """返回世界标识，供 OS 观察发布端分组使用。"""

        return str(self.world.identity["world_id"])

    def command(
        self,
        operation: str,
        _job_key: str,
        identity: Mapping[str, Any],
        parameters: Mapping[str, Any],
    ) -> dict[str, Any]:
        """把 OS 操作名与参数转成一次机器人派发，并保留请求身份用于完成证据。"""

        if not isinstance(identity, Mapping):
            raise TypeError("请求身份必须是对象")
        self._request_identity = deepcopy(dict(identity))
        self._receipt = self.world.command(operation, **dict(parameters))
        self._state = None
        return deepcopy(self._receipt)

    def accept_state(self, state: Mapping[str, Any]) -> None:
        """接收一个已确认扫描的观察；不从中推导物理量。"""

        if not isinstance(state, Mapping):
            raise TypeError("扫描观察必须是对象")
        self._state = deepcopy(dict(state))

    def pending_effect(self) -> None:
        """机器人动作没有库存效果；结算只发生在 OS 完成判据之后。"""

        return None

    def completion(self) -> dict[str, Any] | None:
        """命令结算且至少一个扫描已观察后才返回完成证据。"""

        if self._receipt is None or self._state is None:
            return None
        if not self._receipt["settled"]:
            return None
        if self._receipt["state"] != CommandState.SUCCEEDED.value:
            raise RuntimeError("robot_command_not_succeeded")
        return {
            "identity": deepcopy(self._request_identity),
            "command_id": self._receipt["command_id"],
            "state": self._receipt["state"],
            "epoch": self._receipt["epoch"],
            "tick": self._state.get("tick"),
            "evidence": "robot_binding_receipt",
        }

    def reset_command(self) -> None:
        """清除当前命令，让 OS 在结算前再取一次最终扫描。"""

        self._receipt = None
        self._state = None

    def close(self) -> None:
        """不关闭注入的运行时绑定；端点租约所有权仍归装配方。"""


def _required(config: Mapping[str, Any], key: str) -> Any:
    """只从显式配置字段读取，缺失时给出字段名。"""

    if key not in config:
        raise ValueError(f"build_device_runtime 缺少显式字段: {key}")
    return config[key]


def _profile_digest(profile: Any) -> str:
    """接受 HardwareProfile 或显式映射，只读取 digest。"""

    if isinstance(profile, Mapping):
        digest = profile.get("digest")
    else:
        digest = getattr(profile, "digest", None)
    return _require_text(digest, "profile.digest")


def build_device_runtime(
    config: Mapping[str, Any],
    materials: Any,
    record: Callable[[Mapping[str, Any]], None],
) -> DeviceRuntime:
    """OS 设备包入口：用显式 profile/identity/命令映射装配 ``DeviceRuntime``。

    参数：显式配置、OS 物料视图与事件记录器。返回：OS ``DeviceRuntime``。
    异常：缺少 binding/model 所需字段或字段非法时关闭失败。

    ``materials`` 仅按 OS 装配签名传入；库存结算由 OS ``settle`` 独占，本世界不读它。
    """

    if not isinstance(config, Mapping):
        raise TypeError("config 必须是对象")
    identity = WorldIdentity.from_mapping(_required(config, "world"))
    digest = _profile_digest(_required(config, "profile"))
    commands = _required(config, "commands")
    binding = _required(config, "binding")
    bindings = _required(config, "bindings")
    world = RobotWorld(
        binding,
        identity,
        hardware_profile_digest=digest,
        commands=commands,
        record=record,
        scan_advance=config.get("scan_advance"),
    )
    model = config["model"] if "model" in config else RobotWorldModel(world)
    try:
        from unilabos.simulation.execution_service import DeviceRuntime
    except ImportError as error:  # pragma: no cover - 依赖缺失时给出明确方向
        raise RuntimeError(
            "build_device_runtime 需要 Uni-Lab-OS 的 unilabos.simulation.execution_service"
        ) from error
    return DeviceRuntime(world=world, model=model, bindings=dict(bindings))
