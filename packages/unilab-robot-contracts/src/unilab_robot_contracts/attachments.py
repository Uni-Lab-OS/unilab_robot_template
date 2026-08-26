"""快换工具与夹持物料共享的运动学附着和碰撞资产合同。"""

from __future__ import annotations

import math
import re
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any

from .geometry import RigidTransform

_DIGEST = re.compile(r"^[0-9a-f]{64}$")
_COLLISION_SHAPE_DIMENSIONS = {
    "box": 3,
    "sphere": 1,
    "cylinder": 2,
    "cone": 2,
}


class AttachmentKind(str, Enum):
    """附着子对象的业务类别；不决定视觉模型格式。"""

    TOOL = "tool"
    MATERIAL_PAYLOAD = "material_payload"


class AttachmentState(str, Enum):
    """运行时父子关系的确定状态。"""

    ATTACHED = "attached"
    DETACHED = "detached"
    DETACHING = "detaching"
    UNCERTAIN = "uncertain"


class AttachmentEvidence(str, Enum):
    """附着结论的证据等级。"""

    OBSERVED = "observed"
    CONTROLLER_CONFIRMED = "controller_confirmed"
    NONE = "none"


class AnchorKind(str, Enum):
    """父对象使用根节点还是精确 link 作为运动学锚点。"""

    ROOT = "root"
    LINK = "link"


@dataclass(frozen=True, slots=True)
class KinematicAnchor:
    """附着父对象内的稳定符号锚点。"""

    kind: AnchorKind
    link_name: str | None = None

    def __post_init__(self) -> None:
        """保证 root/link 形态互斥且 link 使用稳定非空名称。"""

        if self.kind is AnchorKind.ROOT:
            if self.link_name is not None:
                raise ValueError("root anchor 不得携带 link_name")
            return
        if self.link_name is None or not self.link_name.strip():
            raise ValueError("link anchor 必须包含 link_name")

    def as_dict(self) -> dict[str, str]:
        """编码不包含模型格式的 wire 形状。"""

        result = {"kind": self.kind.value}
        if self.link_name is not None:
            result["link_name"] = self.link_name
        return result


@dataclass(frozen=True, slots=True)
class CollisionPrimitive:
    """经审批的静态保守碰撞基元。"""

    shape: str
    size_m: tuple[float, ...]
    pose: RigidTransform

    def __post_init__(self) -> None:
        """校验 MoveIt 支持的形状、维度数量和正尺寸。"""

        shape = self.shape.strip().lower()
        expected = _COLLISION_SHAPE_DIMENSIONS.get(shape)
        values = tuple(float(value) for value in self.size_m)
        if expected is None:
            raise ValueError(f"不支持的碰撞基元: {self.shape}")
        if len(values) != expected or any(
            not math.isfinite(value) or value <= 0.0 for value in values
        ):
            raise ValueError(f"{shape} 的 size_m 必须包含 {expected} 个正有限数")
        object.__setattr__(self, "shape", shape)
        object.__setattr__(self, "size_m", values)

    def as_dict(self) -> dict[str, Any]:
        """编码 ToolContext/PayloadCollisionProfile 共用的规划形状。"""

        return {
            "shape": self.shape,
            "size_m": list(self.size_m),
            "pose": {
                "xyz_m": list(self.pose.translation_m),
                "orientation_xyzw": list(self.pose.orientation_xyzw),
            },
        }


@dataclass(frozen=True, slots=True)
class PayloadCollisionProfile:
    """物料/载架进入 MoveIt 时使用的版本化碰撞与负载资产。"""

    profile_ref: str
    digest: str
    mass_kg: float
    center_of_mass_m: tuple[float, float, float]
    root_to_grasp: RigidTransform
    collision_asset_ref: str | None = None
    collision_asset_digest: str | None = None
    collision_asset_scale: float = 1.0
    collision_primitives: tuple[CollisionPrimitive, ...] = ()

    def __post_init__(self) -> None:
        """拒绝无身份、无几何、非法质量或不可审计摘要。"""

        if not self.profile_ref.strip() or not _DIGEST.fullmatch(self.digest):
            raise ValueError("PayloadCollisionProfile 必须包含稳定引用和 SHA-256")
        if not math.isfinite(self.mass_kg) or self.mass_kg < 0.0:
            raise ValueError("PayloadCollisionProfile.mass_kg 必须是非负有限数")
        center = tuple(float(value) for value in self.center_of_mass_m)
        if len(center) != 3 or not all(math.isfinite(value) for value in center):
            raise ValueError("PayloadCollisionProfile.center_of_mass_m 必须包含三个有限数")
        asset = (self.collision_asset_ref or "").strip()
        asset_digest = (self.collision_asset_digest or "").strip()
        asset_scale = float(self.collision_asset_scale)
        primitives = tuple(self.collision_primitives)
        if not asset and not primitives:
            raise ValueError("PayloadCollisionProfile 必须包含碰撞资产或保守基元")
        if bool(asset) != bool(asset_digest):
            raise ValueError("碰撞资产引用和 collision_asset_digest 必须同时提供")
        if asset_digest and _DIGEST.fullmatch(asset_digest) is None:
            raise ValueError("collision_asset_digest 必须是 SHA-256")
        if not math.isfinite(asset_scale) or asset_scale <= 0.0:
            raise ValueError("collision_asset_scale 必须是正有限数")
        object.__setattr__(self, "center_of_mass_m", center)
        object.__setattr__(self, "collision_asset_ref", asset or None)
        object.__setattr__(self, "collision_asset_digest", asset_digest or None)
        object.__setattr__(self, "collision_asset_scale", asset_scale)
        object.__setattr__(self, "collision_primitives", primitives)

    def as_planning_scene(self) -> dict[str, Any]:
        """编码供 MoveIt Adapter 消费的静态碰撞与负载描述。"""

        result: dict[str, Any] = {
            "payload_profile_ref": self.profile_ref,
            "payload_profile_digest": self.digest,
            "mass_kg": self.mass_kg,
            "center_of_mass_m": list(self.center_of_mass_m),
            "root_to_grasp": {
                "xyz_m": list(self.root_to_grasp.translation_m),
                "orientation_xyzw": list(self.root_to_grasp.orientation_xyzw),
            },
            "collision_primitives": [
                primitive.as_dict() for primitive in self.collision_primitives
            ],
        }
        if self.collision_asset_ref is not None:
            result["collision_asset_ref"] = self.collision_asset_ref
            result["collision_asset_digest"] = self.collision_asset_digest
            result["collision_asset_scale"] = self.collision_asset_scale
        return result


@dataclass(frozen=True, slots=True)
class AttachmentCapabilities:
    """Adapter 对快换、负载和碰撞证据的诚实能力声明。"""

    tool_identity_sensor: bool = False
    lock_sensor: bool = False
    payload_sensor: bool = False
    exact_completion_generation: bool = False
    payload_collision: bool = False


@dataclass(frozen=True, slots=True)
class KinematicAttachmentProjection:
    """不携带模型格式、只描述运行时父子 frame 的 latest 投影。"""

    kind: AttachmentKind
    child_ref: str
    parent_ref: str
    anchor: KinematicAnchor
    local_pose: RigidTransform
    state: AttachmentState
    evidence: AttachmentEvidence
    attachment_generation: int
    observed_at: float
    stale_after_s: float
    source: str
    source_boot_id: str
    monotonic_sequence: int
    command_ref: str | None = None
    job_ref: str | None = None
    context_digest: str | None = None
    schema_version: int = 1

    def __post_init__(self) -> None:
        """校验 latest 身份、代次、新鲜度和状态/证据组合。"""

        for name, value in (
            ("child_ref", self.child_ref),
            ("parent_ref", self.parent_ref),
            ("source", self.source),
            ("source_boot_id", self.source_boot_id),
        ):
            if not value.strip():
                raise ValueError(f"KinematicAttachmentProjection.{name} 不能为空")
        if self.schema_version != 1:
            raise ValueError("只支持 kinematic attachment schema_version=1")
        if isinstance(self.attachment_generation, bool) or self.attachment_generation < 1:
            raise ValueError("attachment_generation 必须是正整数")
        if isinstance(self.monotonic_sequence, bool) or self.monotonic_sequence < 1:
            raise ValueError("monotonic_sequence 必须是正整数")
        if not math.isfinite(self.observed_at) or not math.isfinite(self.stale_after_s):
            raise ValueError("附着观测时间和 TTL 必须是有限数")
        if self.stale_after_s <= 0.0:
            raise ValueError("stale_after_s 必须为正")
        if self.state is AttachmentState.UNCERTAIN:
            if self.evidence is not AttachmentEvidence.NONE:
                raise ValueError("uncertain attachment 不得伪造完成证据")
        elif self.evidence is AttachmentEvidence.NONE:
            raise ValueError("确定附着状态必须携带证据等级")
        if self.context_digest is not None and not _DIGEST.fullmatch(
            self.context_digest
        ):
            raise ValueError("context_digest 必须是 SHA-256")

    def is_fresh(self, now: float | None = None) -> bool:
        """判断投影是否仍在同一实时窗口内。"""

        checked_at = time.time() if now is None else float(now)
        age = checked_at - self.observed_at
        return 0.0 <= age <= self.stale_after_s

    def as_dict(self) -> dict[str, Any]:
        """编码 OS DeviceTelemetry 与 FE scene-runtime 共用形状。"""

        result: dict[str, Any] = {
            "schema_version": self.schema_version,
            "kind": self.kind.value,
            "child_ref": self.child_ref,
            "parent_ref": self.parent_ref,
            "anchor": self.anchor.as_dict(),
            "local_pose": {
                "xyz_m": list(self.local_pose.translation_m),
                "orientation_xyzw": list(self.local_pose.orientation_xyzw),
            },
            "state": self.state.value,
            "evidence": self.evidence.value,
            "attachment_generation": self.attachment_generation,
            "observed_at": self.observed_at,
            "stale_after_s": self.stale_after_s,
            "source": self.source,
            "source_boot_id": self.source_boot_id,
            "monotonic_sequence": self.monotonic_sequence,
        }
        for field, value in (
            ("command_ref", self.command_ref),
            ("job_ref", self.job_ref),
            ("context_digest", self.context_digest),
        ):
            if value is not None:
                result[field] = value
        return result


__all__ = [
    "AnchorKind",
    "AttachmentCapabilities",
    "AttachmentEvidence",
    "AttachmentKind",
    "AttachmentState",
    "CollisionPrimitive",
    "KinematicAnchor",
    "KinematicAttachmentProjection",
    "PayloadCollisionProfile",
]
