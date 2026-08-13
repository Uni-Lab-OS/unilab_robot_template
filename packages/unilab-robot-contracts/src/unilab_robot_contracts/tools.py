"""夹爪、快换工具与 MoveIt ToolContext 的厂商无关合同。"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from .commands import CommandResult
from .geometry import RigidTransform
from .observations import ObservationState
from .targets import ToolContext


@dataclass(frozen=True, slots=True)
class ToolDefinition:
    """可更换末端工具的固有模型、TCP 和碰撞资产。"""

    tool_ref: str
    model_digest: str
    mount_to_tcp: RigidTransform
    collision_asset_ref: str

    def __post_init__(self) -> None:
        """校验工具稳定身份、模型摘要和碰撞资产引用。"""

        if not self.tool_ref.strip() or not self.collision_asset_ref.strip():
            raise ValueError("ToolDefinition 必须包含工具和碰撞资产引用")
        if len(self.model_digest) != 64:
            raise ValueError("ToolDefinition.model_digest 必须是 SHA-256")

    def context(self, attachment_generation: int) -> ToolContext:
        """从已确认附着代次生成不可复用的 ToolContext。"""

        if isinstance(attachment_generation, bool) or attachment_generation < 1:
            raise ValueError("工具附着代次必须是正整数")
        payload = {
            "tool_ref": self.tool_ref,
            "model_digest": self.model_digest,
            "mount_to_tcp": {
                "xyz_m": self.mount_to_tcp.translation_m,
                "orientation_xyzw": self.mount_to_tcp.orientation_xyzw,
            },
            "collision_asset_ref": self.collision_asset_ref,
            "attachment_generation": attachment_generation,
        }
        digest = hashlib.sha256(
            json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()
        return ToolContext(
            context_id=f"tool:{self.tool_ref}",
            digest=digest,
            mount_to_tcp=self.mount_to_tcp,
            attachment_generation=attachment_generation,
        )


@dataclass(frozen=True, slots=True)
class ToolAttachmentObservation:
    """快换机构对当前工具身份、代次和锁紧状态的观测。"""

    state: ObservationState
    observed_at: float
    max_age_s: float
    source: str
    tool_ref: str | None
    attachment_generation: int
    locked: bool | None

    def __post_init__(self) -> None:
        """拒绝无来源、无效时间和含糊的 known 工具状态。"""

        if not math.isfinite(self.observed_at) or self.max_age_s <= 0.0:
            raise ValueError("工具观测必须包含有效时间和新鲜度")
        if not self.source.strip() or self.attachment_generation < 0:
            raise ValueError("工具观测来源或附着代次无效")
        if self.state is ObservationState.KNOWN and self.locked is None:
            raise ValueError("known 工具观测必须明确 locked")


@dataclass(frozen=True, slots=True)
class GripperObservation:
    """夹爪开合和负载状态的只读观测。"""

    state: ObservationState
    observed_at: float
    max_age_s: float
    source: str
    closed: bool | None
    holding_payload: bool | None


@runtime_checkable
class EndEffectorPort(Protocol):
    """独立于机械臂型号和运输后端的夹爪端口。"""

    def observe(self) -> GripperObservation:
        """读取夹爪和负载观测。"""

    def grip(
        self, command_id: str, *, payload_profile: str
    ) -> CommandResult:
        """抓取指定负载类型并返回完成见证。"""

    def release(self, command_id: str) -> CommandResult:
        """释放当前负载并返回完成见证。"""


@runtime_checkable
class ToolChangerPort(Protocol):
    """独立快换端口；工具代次变化会使旧 ToolContext 失效。"""

    def observe(self) -> ToolAttachmentObservation:
        """读取当前工具身份和锁紧代次。"""

    def change_tool(self, command_id: str, *, tool_ref: str) -> CommandResult:
        """更换并锁紧工具，返回精确完成结果。"""

    @property
    def active_tool_context(self) -> ToolContext:
        """返回已由锁紧观测确认的当前 ToolContext。"""


@runtime_checkable
class ToolContextActivator(Protocol):
    """把新工具 TCP/碰撞模型应用到规划器并确认生效。"""

    def activate_tool_context(self, tool_context: ToolContext) -> None:
        """确认 PlanningScene 使用同一摘要和附着代次。"""


__all__ = [
    "EndEffectorPort",
    "GripperObservation",
    "ToolAttachmentObservation",
    "ToolChangerPort",
    "ToolContextActivator",
    "ToolDefinition",
]
