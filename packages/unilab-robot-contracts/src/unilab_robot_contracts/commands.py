"""已由部署资产解析、可交给执行后端的机械臂命令。"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any

from .actions import ActionKind


class CommandState(str, Enum):
    """机械臂命令生命周期；派发歧义使用规范 ``execution_unknown``。"""

    ACCEPTED = "accepted"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"
    REJECTED = "rejected"
    EXECUTION_UNKNOWN = "execution_unknown"

    @property
    def terminal(self) -> bool:
        """返回该状态是否已经完成物理结算。"""

        return self in {
            CommandState.SUCCEEDED,
            CommandState.FAILED,
            CommandState.CANCELED,
            CommandState.REJECTED,
        }


@dataclass(frozen=True)
class MotionSegment:
    """引用已验证运动资产的一个内部段，不暴露任意在线轨迹。"""

    segment_id: str
    target_ref: str
    parameters: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """校验内部段只引用稳定资产。

        参数：无。返回：无。异常：标识或资产引用为空时抛出 ``ValueError``。
        """

        if not self.segment_id.strip() or not self.target_ref.strip():
            raise ValueError("MotionSegment 必须包含 segment_id 与 target_ref")


@dataclass(frozen=True)
class RobotCommand:
    """执行边界命令；禁止携带 Site、Material、原始地址和任意程序入口。"""

    command_id: str
    action: ActionKind
    hardware_profile_digest: str
    payload_profile: str
    source_boot_id: str
    monotonic_sequence: int
    segments: tuple[MotionSegment, ...]
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """校验命令身份、版本和不可为空的运动序列。

        参数：无。返回：无。异常：违反合同或 metadata 泄漏禁用字段时抛出 ``ValueError``。
        """

        required = (
            self.command_id,
            self.hardware_profile_digest,
            self.payload_profile,
            self.source_boot_id,
        )
        if any(not value.strip() for value in required):
            raise ValueError(
                "RobotCommand 的身份、profile、payload 与 boot_id 不能为空"
            )
        if isinstance(self.monotonic_sequence, bool) or self.monotonic_sequence < 1:
            raise ValueError("monotonic_sequence 必须是正整数")
        if not self.segments:
            raise ValueError("RobotCommand 至少包含一个已解析运动段")
        forbidden = {
            "site",
            "site_ref",
            "warehouse",
            "material",
            "resource",
            "presence_variable",
            "legacy_runner_name",
            "opc_address",
        }
        leaked = forbidden.intersection(self.metadata)
        if leaked:
            raise ValueError(
                f"RobotCommand.metadata 泄漏领域或地址字段: {sorted(leaked)}"
            )

    def fingerprint(self) -> str:
        """返回排除运行时钟的稳定请求摘要，用于幂等冲突检测。"""

        payload = json.dumps(
            self.canonical_payload(),
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()

    def canonical_payload(self) -> Mapping[str, Any]:
        """返回可稳定序列化的命令值，用于审计、摘要和测试。"""

        payload = _jsonable(asdict(self))
        if not isinstance(payload, Mapping):
            raise TypeError("RobotCommand canonical payload 必须是对象")
        return payload


@dataclass(frozen=True)
class RailMoveCommand:
    """组合工站内只移动导轨的持久命令；目标必须来自已发布 target-set。"""

    command_id: str
    hardware_profile_digest: str
    source_boot_id: str
    monotonic_sequence: int
    target_ref: str

    def __post_init__(self) -> None:
        """拒绝空身份、空目标和无效序号。"""

        required = (
            self.command_id,
            self.hardware_profile_digest,
            self.source_boot_id,
            self.target_ref,
        )
        if any(not value.strip() for value in required):
            raise ValueError("RailMoveCommand 的身份、profile、boot_id 与 target_ref 不能为空")
        if isinstance(self.monotonic_sequence, bool) or self.monotonic_sequence < 1:
            raise ValueError("monotonic_sequence 必须是正整数")

    def fingerprint(self) -> str:
        """返回包含导轨目标的稳定请求摘要，用于精确幂等。"""

        payload = json.dumps(
            self.canonical_payload(),
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()

    def canonical_payload(self) -> Mapping[str, Any]:
        """返回可稳定序列化的导轨命令值。"""

        payload = _jsonable(asdict(self))
        if not isinstance(payload, Mapping):
            raise TypeError("RailMoveCommand canonical payload 必须是对象")
        return payload


@dataclass(frozen=True)
class CommandResult:
    """执行后端或协调器返回的权威命令投影。"""

    command_id: str
    state: CommandState
    message: str
    output: Mapping[str, Any] = field(default_factory=dict)

    @property
    def success(self) -> bool:
        """仅在物理完成已被确认时返回成功。"""

        return self.state is CommandState.SUCCEEDED


@dataclass(frozen=True)
class PhysicalSettlementEvidence:
    """用于显式解除本地派发阻断的物理结算证据。"""

    command_id: str
    terminal_state: CommandState
    witness_id: str
    source: str

    def __post_init__(self) -> None:
        """拒绝空见证、非终态和没有物理含义的 rejected。"""

        if not self.command_id.strip() or not self.witness_id.strip() or not self.source.strip():
            raise ValueError("物理结算证据必须包含命令、见证和来源")
        if self.terminal_state not in {
            CommandState.SUCCEEDED,
            CommandState.FAILED,
            CommandState.CANCELED,
        }:
            raise ValueError("物理结算证据必须对应物理终态")


def _jsonable(value: Any) -> Any:
    """把合同对象递归转换为稳定 JSON 值。"""

    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value
