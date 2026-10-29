"""夹爪、快换工具与 MoveIt ToolContext 的厂商无关合同。"""

from __future__ import annotations

import hashlib
import json
import math
import re
import time
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any, Protocol, runtime_checkable

from .attachments import CollisionPrimitive
from .commands import CommandResult
from .geometry import RigidTransform
from .observations import ObservationState
from .targets import ToolContext

_DIGEST = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True, slots=True)
class ToolDefinition:
    """可更换末端工具的固有模型、TCP 和碰撞资产。"""

    tool_ref: str
    model_digest: str
    mount_to_tcp: RigidTransform
    collision_asset_ref: str
    visual_asset_ref: str = ""
    collision_digest: str = ""
    flange_frame: str = "tool0"
    tcp_frame: str = "tcp"
    grasp_frame: str = "grasp_frame"
    mass_kg: float = 0.0
    center_of_mass_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    mount_to_visual: RigidTransform = field(default_factory=RigidTransform.identity)
    mount_to_grasp: RigidTransform | None = None
    static_collision_primitives: tuple[CollisionPrimitive, ...] = ()

    def __post_init__(self) -> None:
        """校验工具稳定身份、模型摘要和碰撞资产引用。"""

        if not self.tool_ref.strip() or not self.collision_asset_ref.strip():
            raise ValueError("ToolDefinition 必须包含工具和碰撞资产引用")
        if not _DIGEST.fullmatch(self.model_digest):
            raise ValueError("ToolDefinition.model_digest 必须是 SHA-256")
        visual_asset_ref = self.visual_asset_ref.strip() or self.collision_asset_ref
        collision_digest = self.collision_digest.strip() or self.model_digest
        if not _DIGEST.fullmatch(collision_digest):
            raise ValueError("ToolDefinition.collision_digest 必须是 SHA-256")
        for name, value in (
            ("flange_frame", self.flange_frame),
            ("tcp_frame", self.tcp_frame),
            ("grasp_frame", self.grasp_frame),
        ):
            if not value.strip():
                raise ValueError(f"ToolDefinition.{name} 不能为空")
        if not math.isfinite(self.mass_kg) or self.mass_kg < 0.0:
            raise ValueError("ToolDefinition.mass_kg 必须是非负有限数")
        center = tuple(float(value) for value in self.center_of_mass_m)
        if len(center) != 3 or not all(math.isfinite(value) for value in center):
            raise ValueError("ToolDefinition.center_of_mass_m 必须包含三个有限数")
        object.__setattr__(self, "visual_asset_ref", visual_asset_ref)
        object.__setattr__(self, "collision_digest", collision_digest)
        object.__setattr__(self, "center_of_mass_m", center)
        object.__setattr__(
            self,
            "mount_to_grasp",
            self.mount_to_tcp if self.mount_to_grasp is None else self.mount_to_grasp,
        )
        object.__setattr__(
            self,
            "static_collision_primitives",
            tuple(self.static_collision_primitives),
        )

    def context(self, attachment_generation: int) -> ToolContext:
        """从已确认附着代次生成不可复用的 ToolContext。"""

        if isinstance(attachment_generation, bool) or attachment_generation < 1:
            raise ValueError("工具附着代次必须是正整数")
        mount_to_grasp = self.mount_to_grasp or self.mount_to_tcp
        payload = {
            "tool_ref": self.tool_ref,
            "model_digest": self.model_digest,
            "mount_to_tcp": {
                "xyz_m": self.mount_to_tcp.translation_m,
                "orientation_xyzw": self.mount_to_tcp.orientation_xyzw,
            },
            "collision_asset_ref": self.collision_asset_ref,
            "visual_asset_ref": self.visual_asset_ref,
            "collision_digest": self.collision_digest,
            "flange_frame": self.flange_frame,
            "tcp_frame": self.tcp_frame,
            "grasp_frame": self.grasp_frame,
            "mass_kg": self.mass_kg,
            "center_of_mass_m": self.center_of_mass_m,
            "mount_to_visual": {
                "xyz_m": self.mount_to_visual.translation_m,
                "orientation_xyzw": self.mount_to_visual.orientation_xyzw,
            },
            "mount_to_grasp": {
                "xyz_m": mount_to_grasp.translation_m,
                "orientation_xyzw": mount_to_grasp.orientation_xyzw,
            },
            "collision_primitives": [
                primitive.as_dict() for primitive in self.static_collision_primitives
            ],
            "attachment_generation": attachment_generation,
        }
        digest = hashlib.sha256(
            json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()
        return ToolContext(
            context_id=self.tool_ref,
            digest=digest,
            mount_to_tcp=self.mount_to_tcp,
            attachment_generation=attachment_generation,
            planning_scene={
                "attached_body_id": self.tool_ref,
                "parent_link": self.flange_frame,
                "allowed_touch_links": [self.flange_frame],
                "collision_asset_ref": self.collision_asset_ref,
                "model_digest": self.model_digest,
                "collision_digest": self.collision_digest,
                "collision_primitives": [
                    primitive.as_dict()
                    for primitive in self.static_collision_primitives
                ],
            },
        )


@dataclass(frozen=True, slots=True)
class ToolContextActivationReceipt:
    """MoveIt 对同一工具摘要和附着代次的结构化回读见证。"""

    applied: bool
    tool_context_digest: str
    attachment_generation: int
    observed_at: float
    source: str

    def __post_init__(self) -> None:
        """拒绝无来源、无效摘要、代次或时间。"""

        if not _DIGEST.fullmatch(self.tool_context_digest):
            raise ValueError("ToolContextActivationReceipt.digest 必须是 SHA-256")
        if self.attachment_generation < 1:
            raise ValueError("ToolContextActivationReceipt.attachment_generation 无效")
        if not math.isfinite(self.observed_at) or not self.source.strip():
            raise ValueError("ToolContextActivationReceipt 必须包含时间和来源")

    @classmethod
    def from_mapping(
        cls,
        value: Mapping[str, Any],
        *,
        source: str,
    ) -> ToolContextActivationReceipt:
        """把厂商 Adapter 的结构化确认规范化为稳定值对象。"""

        return cls(
            applied=value.get("applied") is True,
            tool_context_digest=str(value.get("tool_context_digest", "")),
            attachment_generation=int(value.get("attachment_generation", 0)),
            observed_at=time.time(),
            source=source,
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

    def grip(self, command_id: str, *, payload_profile: str) -> CommandResult:
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

    def activate_tool_context(
        self,
        tool_context: ToolContext,
    ) -> ToolContextActivationReceipt:
        """确认 PlanningScene 使用同一摘要和附着代次。"""


__all__ = [
    "EndEffectorPort",
    "GripperObservation",
    "ToolAttachmentObservation",
    "ToolChangerPort",
    "ToolContextActivationReceipt",
    "ToolContextActivator",
    "ToolDefinition",
]
