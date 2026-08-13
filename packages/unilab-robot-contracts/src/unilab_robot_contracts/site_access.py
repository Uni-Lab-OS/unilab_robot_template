"""作者层 Site 定位、启动解析和激活快照合同。"""

from __future__ import annotations

import re
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from enum import Enum
from typing import Any

from .actions import ActionKind, SiteRef
from .point_set_validation import stable_digest

_DIGEST = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True, slots=True)
class SiteLocator:
    """领域作者稳定保存的资源定义引用与局部 slot key。"""

    owner_resource_ref: str
    slot_key: str

    def __post_init__(self) -> None:
        """禁止空引用和看起来像环境实例 UUID 的作者值。"""

        if not self.owner_resource_ref.strip() or not self.slot_key.strip():
            raise ValueError("SiteLocator 必须包含 owner_resource_ref 与 slot_key")
        if _looks_like_uuid(self.owner_resource_ref) or _looks_like_uuid(self.slot_key):
            raise ValueError("领域作者资产不得固化环境实例 UUID")

    @property
    def canonical(self) -> str:
        """返回用于唯一匹配拓扑的稳定作者键。"""

        return f"{self.owner_resource_ref}:{self.slot_key}"


@dataclass(frozen=True, slots=True)
class TopologySite:
    """OS 一次拓扑快照中可由作者键定位的实际 Site。"""

    site_uuid: str
    owner_resource_ref: str
    slot_key: str

    def __post_init__(self) -> None:
        """要求拓扑实例身份与两个稳定索引均非空。"""

        if any(
            not value.strip()
            for value in (self.site_uuid, self.owner_resource_ref, self.slot_key)
        ):
            raise ValueError("TopologySite 字段不得为空")

    @property
    def locator(self) -> SiteLocator:
        """投影为不携带实例 UUID 的作者定位键。"""

        return SiteLocator(self.owner_resource_ref, self.slot_key)


@dataclass(frozen=True, slots=True)
class SiteTopologySnapshot:
    """一次启动或重新上传后由 OS 提供的只读 Site 拓扑。"""

    epoch: str
    digest: str
    sites: tuple[TopologySite, ...]

    def __post_init__(self) -> None:
        """验证拓扑身份、摘要和无重复 UUID。"""

        if not self.epoch.strip() or _DIGEST.fullmatch(self.digest.lower()) is None:
            raise ValueError("SiteTopologySnapshot 必须包含 epoch 与 SHA-256 digest")
        uuids = [site.site_uuid for site in self.sites]
        if len(uuids) != len(set(uuids)):
            raise ValueError("SiteTopologySnapshot 含重复 Site UUID")


class SiteExecutionKind(str, Enum):
    """一个 Site 动作最终使用几何点位还是 PLC 整块程序。"""

    POINT_TARGET = "point_target"
    PLC_PROGRAM = "plc_program"


@dataclass(frozen=True, slots=True)
class SiteOperationDeclaration:
    """一个通用动作到 PointSet 或 PLCProgramSet 的作者层引用。"""

    action: ActionKind
    kind: SiteExecutionKind
    execution_ref: str
    motion_policy_ref: str | None
    parameters: Mapping[str, Any]
    qualification_ref: str

    def __post_init__(self) -> None:
        """验证后端无关动作和受限执行引用。"""

        if not self.execution_ref.strip() or not self.qualification_ref.strip():
            raise ValueError(
                "SiteOperationDeclaration execution_ref 与 qualification_ref 不能为空"
            )
        if self.kind is SiteExecutionKind.POINT_TARGET and not (
            self.motion_policy_ref and self.motion_policy_ref.strip()
        ):
            raise ValueError("point_target 操作必须引用 AccessMotionPolicy")
        if self.kind is SiteExecutionKind.PLC_PROGRAM and self.motion_policy_ref:
            raise ValueError("PLC whole-block 操作不得伪装成在线 MotionProfile")


@dataclass(frozen=True, slots=True)
class SiteAccessDeclaration:
    """一个作者 Site 定位键的负载准入与通用动作声明。"""

    declaration_ref: str
    locator: SiteLocator
    operations: Mapping[ActionKind, SiteOperationDeclaration]
    allowed_payload_profiles: frozenset[str]
    occupancy_observation_ref: str


@dataclass(frozen=True, slots=True)
class ResolvedSiteAccessBinding:
    """启动时把作者定位键精确解析到当前 Site UUID 的结果。"""

    declaration_ref: str
    site_ref: SiteRef
    locator: SiteLocator
    operations: Mapping[ActionKind, SiteOperationDeclaration]
    allowed_payload_profiles: frozenset[str]
    occupancy_observation_ref: str


@dataclass(frozen=True, slots=True)
class SiteAccessActivation:
    """一次任务必须冻结使用的声明、拓扑和解析结果快照。"""

    activation_id: str
    declarations_revision: str
    declarations_digest: str
    topology_epoch: str
    topology_digest: str
    bindings: Mapping[str, ResolvedSiteAccessBinding]

    @classmethod
    def prepare(
        cls,
        declarations: Mapping[str, Any],
        *,
        declarations_digest: str,
        topology: SiteTopologySnapshot,
        point_target_refs: Iterable[str] = (),
        plc_program_refs: Iterable[str] = (),
    ) -> SiteAccessActivation:
        """关闭失败地解析全部作者键并冻结当前拓扑 UUID。

        参数：声明资产及原始摘要、OS 拓扑、当前激活可用的两类执行引用。
        返回：候选激活快照。异常：零匹配、多匹配或引用缺失时拒绝准备。
        """

        revision, parsed = parse_site_access_declarations(declarations)
        if _DIGEST.fullmatch(declarations_digest.lower()) is None:
            raise ValueError("declarations_digest 必须是 SHA-256")
        point_refs = frozenset(point_target_refs)
        program_refs = frozenset(plc_program_refs)
        topology_index: dict[str, list[TopologySite]] = {}
        for site in topology.sites:
            topology_index.setdefault(site.locator.canonical, []).append(site)
        resolved: dict[str, ResolvedSiteAccessBinding] = {}
        for declaration in parsed:
            matches = topology_index.get(declaration.locator.canonical, [])
            if len(matches) != 1:
                raise ValueError(
                    f"{declaration.declaration_ref} 的 SiteLocator 必须精确匹配一次，"
                    f"实际 {len(matches)} 次"
                )
            _validate_execution_refs(declaration, point_refs, program_refs)
            resolved[declaration.declaration_ref] = ResolvedSiteAccessBinding(
                declaration.declaration_ref,
                SiteRef(matches[0].site_uuid),
                declaration.locator,
                declaration.operations,
                declaration.allowed_payload_profiles,
                declaration.occupancy_observation_ref,
            )
        identity_payload = {
            "declarations_revision": revision,
            "declarations_digest": declarations_digest.lower(),
            "topology_epoch": topology.epoch,
            "topology_digest": topology.digest.lower(),
            "bindings": {
                key: value.site_ref.site_uuid for key, value in sorted(resolved.items())
            },
        }
        return cls(
            stable_digest(identity_payload),
            revision,
            declarations_digest.lower(),
            topology.epoch,
            topology.digest.lower(),
            resolved,
        )

    def binding_for_site(self, site_uuid: str) -> ResolvedSiteAccessBinding:
        """按当前任务冻结的 Site UUID 返回唯一绑定。"""

        matches = [
            binding
            for binding in self.bindings.values()
            if binding.site_ref.site_uuid == site_uuid
        ]
        if len(matches) != 1:
            raise ValueError(f"激活快照中 Site UUID 匹配数量不是 1: {site_uuid}")
        return matches[0]


def parse_site_access_declarations(
    data: Mapping[str, Any],
) -> tuple[str, tuple[SiteAccessDeclaration, ...]]:
    """解析不含 UUID、坐标、程序号或控制器地址的作者资产。"""

    if data.get("schema") != "unilab.site-access-declarations/v1":
        raise ValueError(
            "site access schema 必须为 unilab.site-access-declarations/v1"
        )
    revision = str(data.get("revision", "")).strip()
    if not revision:
        raise ValueError("SiteAccessDeclaration revision 不能为空")
    items = data.get("declarations")
    if not isinstance(items, list):
        raise TypeError("declarations 必须是列表")
    parsed: list[SiteAccessDeclaration] = []
    seen_refs: set[str] = set()
    seen_locators: set[str] = set()
    for value in items:
        if not isinstance(value, Mapping):
            raise TypeError("declaration 必须是对象")
        forbidden = {
            "site_uuid",
            "warehouse_id",
            "site_id",
            "program_number",
            "opc_address",
        }.intersection(value)
        if forbidden:
            raise ValueError(f"作者 Site 声明含禁用实例/Adapter 字段: {sorted(forbidden)}")
        declaration = _parse_declaration(value)
        if declaration.declaration_ref in seen_refs:
            raise ValueError(f"重复 declaration_ref: {declaration.declaration_ref}")
        if declaration.locator.canonical in seen_locators:
            raise ValueError(f"重复 SiteLocator: {declaration.locator.canonical}")
        seen_refs.add(declaration.declaration_ref)
        seen_locators.add(declaration.locator.canonical)
        parsed.append(declaration)
    return revision, tuple(parsed)


def _parse_declaration(data: Mapping[str, Any]) -> SiteAccessDeclaration:
    """把单条作者对象转换为类型化声明。"""

    locator = SiteLocator(str(data["owner_resource_ref"]), str(data["slot_key"]))
    action_data = data.get("operations")
    if not isinstance(action_data, Mapping) or not action_data:
        raise TypeError("SiteAccessDeclaration.operations 必须是非空对象")
    operations: dict[ActionKind, SiteOperationDeclaration] = {}
    for action_name, operation_value in action_data.items():
        action = ActionKind(str(action_name))
        if not isinstance(operation_value, Mapping):
            raise TypeError(f"operations.{action.value} 必须是对象")
        kind = SiteExecutionKind(str(operation_value["kind"]))
        operation = SiteOperationDeclaration(
            action,
            kind,
            str(operation_value["ref"]),
            (
                None
                if operation_value.get("motion_policy_ref") is None
                else str(operation_value["motion_policy_ref"])
            ),
            dict(operation_value.get("parameters") or {}),
            str(operation_value.get("qualification_ref", "")),
        )
        operations[action] = operation
    payloads = frozenset(str(item) for item in data.get("allowed_payload_profiles", ()))
    observation = str(data.get("occupancy_observation_ref", "")).strip()
    if not payloads or not observation:
        raise ValueError("SiteAccessDeclaration 必须包含负载准入与占用观测引用")
    return SiteAccessDeclaration(
        str(data["declaration_ref"]),
        locator,
        operations,
        payloads,
        observation,
    )


def _validate_execution_refs(
    declaration: SiteAccessDeclaration,
    point_refs: frozenset[str],
    program_refs: frozenset[str],
) -> None:
    """要求声明中的执行引用存在于同一次候选激活。"""

    for operation in declaration.operations.values():
        available = (
            point_refs
            if operation.kind is SiteExecutionKind.POINT_TARGET
            else program_refs
        )
        if operation.execution_ref not in available:
            raise ValueError(
                f"{declaration.declaration_ref}.{operation.action.value} 引用缺失: "
                f"{operation.execution_ref}"
            )


def _looks_like_uuid(value: str) -> bool:
    """保守识别带连字符的 128-bit UUID，防止固化环境实例身份。"""

    return bool(
        re.fullmatch(
            r"[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-"
            r"[0-9a-fA-F]{4}-[0-9a-fA-F]{12}",
            value.strip(),
        )
    )


__all__ = [
    "ResolvedSiteAccessBinding",
    "SiteAccessActivation",
    "SiteAccessDeclaration",
    "SiteExecutionKind",
    "SiteLocator",
    "SiteOperationDeclaration",
    "SiteTopologySnapshot",
    "TopologySite",
    "parse_site_access_declarations",
]
