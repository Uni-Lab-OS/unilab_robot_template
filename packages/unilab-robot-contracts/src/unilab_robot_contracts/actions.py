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
    """稳定引用一个 Warehouse 下的库位（Site），不携带坐标或地址。"""

    warehouse_id: str
    site_id: str

    def __post_init__(self) -> None:
        """校验 Warehouse 与 Site 身份。

        参数：无；校验当前实例。返回：无。异常：任一身份为空时抛出 ``ValueError``。
        """

        if not self.warehouse_id.strip() or not self.site_id.strip():
            raise ValueError("SiteRef 必须同时包含 warehouse_id 与 site_id")

    @property
    def canonical(self) -> str:
        """返回可用于部署解析的稳定引用，不包含物理控制细节。"""

        return f"{self.warehouse_id}:{self.site_id}"


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
