"""Workflow/FE 可见的厂商无关机械臂动作合同。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ActionKind(str, Enum):
    """公开动作类型；不得编码 PLC、SDK 或 MoveIt 后端。"""

    PICK = "pick"
    PLACE = "place"
    POUR = "pour"


@dataclass(frozen=True)
class ResourceSlotRef:
    """引用由 WorkflowTask 冻结的物料占位符（ResourceSlot）。"""

    slot_id: str

    def __post_init__(self) -> None:
        """校验占位符身份非空。

        参数：无；校验当前实例。返回：无。异常：身份为空时抛出 ``ValueError``。
        """

        if not self.slot_id.strip():
            raise ValueError("ResourceSlotRef.slot_id 不能为空")


@dataclass(frozen=True)
class SiteRef:
    """引用一次 OS 激活快照中的规范库位（Site）UUID。"""

    site_uuid: str

    def __post_init__(self) -> None:
        """校验运行期 Site UUID 非空。

        参数：无；校验当前实例。返回：无。异常：身份为空时抛出 ``ValueError``。
        """

        if not self.site_uuid.strip():
            raise ValueError("SiteRef.site_uuid 不能为空")

    @property
    def canonical(self) -> str:
        """返回冻结任务可持有的运行期规范引用。"""

        return self.site_uuid


@dataclass(frozen=True)
class PickAction:
    """从来源库位执行物理取料。"""

    resource: ResourceSlotRef
    source_site: SiteRef


@dataclass(frozen=True)
class PlaceAction:
    """向目标库位执行物理放料。"""

    resource: ResourceSlotRef
    target_site: SiteRef


@dataclass(frozen=True)
class PourAction:
    """在交互库位执行两个物料间的不可分割倾倒过程。"""

    source_resource: ResourceSlotRef
    target_resource: ResourceSlotRef
    interaction_site: SiteRef
