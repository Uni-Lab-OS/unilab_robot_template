"""点位、PLC 程序和 Site 操作三类独立资格记录与激活门禁。"""

from __future__ import annotations

import re
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from enum import Enum
from typing import Any

from .actions import ActionKind
from .hardware_profile import DeploymentMode

_DIGEST = re.compile(r"^[0-9a-f]{64}$")


class QualificationScope(str, Enum):
    """资格证据的三种不可互相替代范围。"""

    POINT = "point"
    PROGRAM = "program"
    SITE_OPERATION = "site_operation"


@dataclass(frozen=True, slots=True)
class PointQualification:
    """绑定 PointSet、目标解析摘要和全部环境资产的低速见证。"""

    qualification_ref: str
    point_set_digest: str
    target_ref: str
    resolved_target_digest: str
    arm_model_digest: str
    tool_context_digest: str
    installation_calibration_digest: str
    collision_environment_digest: str
    evidence_ref: str
    approved_by: str
    approved_for: frozenset[DeploymentMode]

    def __post_init__(self) -> None:
        """要求全部身份、摘要与证据完整，禁止“整个文件已测”式模糊资格。"""

        _validate_common(
            self.qualification_ref,
            self.evidence_ref,
            self.approved_by,
            self.approved_for,
            (
                self.point_set_digest,
                self.resolved_target_digest,
                self.arm_model_digest,
                self.tool_context_digest,
                self.installation_calibration_digest,
                self.collision_environment_digest,
            ),
        )
        if not self.target_ref.strip():
            raise ValueError("PointQualification.target_ref 不能为空")


@dataclass(frozen=True, slots=True)
class ProgramQualification:
    """绑定 PLCProgramSet、程序引用和 PLC Adapter 的整块程序见证。"""

    qualification_ref: str
    program_set_digest: str
    program_ref: str
    plc_adapter_digest: str
    evidence_ref: str
    approved_by: str
    approved_for: frozenset[DeploymentMode]

    def __post_init__(self) -> None:
        """要求程序身份、两份资产摘要和验收证据完整。"""

        _validate_common(
            self.qualification_ref,
            self.evidence_ref,
            self.approved_by,
            self.approved_for,
            (self.program_set_digest, self.plc_adapter_digest),
        )
        if not self.program_ref.strip():
            raise ValueError("ProgramQualification.program_ref 不能为空")


@dataclass(frozen=True, slots=True)
class SiteOperationQualification:
    """绑定一个当前 Site 作者键、动作、负载和基础资格的端到端见证。"""

    qualification_ref: str
    declarations_digest: str
    declaration_ref: str
    action: ActionKind
    payload_profiles: frozenset[str]
    execution_qualification_ref: str
    motion_profiles_digest: str | None
    occupancy_observation_ref: str
    evidence_ref: str
    approved_by: str
    approved_for: frozenset[DeploymentMode]

    def __post_init__(self) -> None:
        """要求 Site 操作证据精确绑定作者声明和底层执行资格。"""

        _validate_common(
            self.qualification_ref,
            self.evidence_ref,
            self.approved_by,
            self.approved_for,
            (
                self.declarations_digest,
                *(
                    (self.motion_profiles_digest,)
                    if self.motion_profiles_digest
                    else ()
                ),
            ),
        )
        required = (
            self.declaration_ref,
            self.execution_qualification_ref,
            self.occupancy_observation_ref,
        )
        if any(not value.strip() for value in required) or not self.payload_profiles:
            raise ValueError("SiteOperationQualification 绑定字段不得为空")
        if any(not value.strip() for value in self.payload_profiles):
            raise ValueError("SiteOperationQualification.payload_profiles 含空值")


@dataclass(frozen=True, slots=True)
class QualificationCatalog:
    """一次候选激活中三类资格记录的唯一查找入口。"""

    points: Mapping[str, PointQualification]
    programs: Mapping[str, ProgramQualification]
    site_operations: Mapping[str, SiteOperationQualification]

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> QualificationCatalog:
        """解析 v1 资格集合，并拒绝跨类型重复 identity。"""

        if data.get("schema") != "unilab.robot-qualifications/v1":
            raise ValueError(
                "qualification schema 必须为 unilab.robot-qualifications/v1"
            )
        points = _record_index(data.get("points", ()), _point_record)
        programs = _record_index(data.get("programs", ()), _program_record)
        site_operations = _record_index(
            data.get("site_operations", ()),
            _site_operation_record,
        )
        all_refs = [*points, *programs, *site_operations]
        if len(all_refs) != len(set(all_refs)):
            raise ValueError("三类 Qualification 不得复用 qualification_ref")
        return cls(points, programs, site_operations)

    def require_site_operation(
        self,
        qualification_ref: str,
        *,
        mode: DeploymentMode,
        declarations_digest: str,
        declaration_ref: str,
        action: ActionKind,
        payload_profile: str,
        occupancy_observation_ref: str,
        motion_profiles_digest: str | None,
    ) -> SiteOperationQualification:
        """精确核对 Site 操作身份，并要求基础资格批准当前模式。"""

        try:
            qualification = self.site_operations[qualification_ref]
        except KeyError as exc:
            raise ValueError(
                f"缺少 SiteOperationQualification: {qualification_ref}"
            ) from exc
        _require_mode(qualification.qualification_ref, qualification.approved_for, mode)
        expected = (
            qualification.declarations_digest == declarations_digest,
            qualification.declaration_ref == declaration_ref,
            qualification.action is action,
            payload_profile in qualification.payload_profiles,
            qualification.occupancy_observation_ref == occupancy_observation_ref,
        )
        if not all(expected):
            raise ValueError(
                f"{qualification_ref} 与当前声明、动作、负载或占用观测不匹配"
            )
        base = self.points.get(qualification.execution_qualification_ref)
        if base is None:
            base = self.programs.get(qualification.execution_qualification_ref)
        if base is None:
            raise ValueError(
                "SiteOperationQualification 引用缺失 Point/ProgramQualification: "
                f"{qualification.execution_qualification_ref}"
            )
        _require_mode(base.qualification_ref, base.approved_for, mode)
        if isinstance(base, PointQualification):
            if (
                motion_profiles_digest is None
                or qualification.motion_profiles_digest != motion_profiles_digest
            ):
                raise ValueError(
                    f"{qualification_ref} 未绑定当前 MotionProfile 资产摘要"
                )
        elif (
            qualification.motion_profiles_digest is not None
            or motion_profiles_digest is not None
        ):
            raise ValueError("PLC SiteOperationQualification 不得绑定 MotionProfile")
        return qualification

    def require_point(
        self,
        qualification_ref: str,
        *,
        mode: DeploymentMode,
        point_set_digest: str,
        target_ref: str,
        resolved_target_digest: str,
        arm_model_digest: str,
        tool_context_digest: str,
        installation_calibration_digest: str,
        collision_environment_digest: str,
    ) -> PointQualification:
        """精确核对一条点位及其全部规划环境摘要。"""

        try:
            qualification = self.points[qualification_ref]
        except KeyError as exc:
            raise ValueError(f"缺少 PointQualification: {qualification_ref}") from exc
        _require_mode(qualification.qualification_ref, qualification.approved_for, mode)
        expected = (
            qualification.point_set_digest == point_set_digest,
            qualification.target_ref == target_ref,
            qualification.resolved_target_digest == resolved_target_digest,
            qualification.arm_model_digest == arm_model_digest,
            qualification.tool_context_digest == tool_context_digest,
            qualification.installation_calibration_digest
            == installation_calibration_digest,
            qualification.collision_environment_digest == collision_environment_digest,
        )
        if not all(expected):
            raise ValueError(f"{qualification_ref} 与当前点位或规划环境摘要不匹配")
        return qualification

    def require_program(
        self,
        qualification_ref: str,
        *,
        mode: DeploymentMode,
        program_set_digest: str,
        program_ref: str,
        plc_adapter_digest: str,
    ) -> ProgramQualification:
        """精确核对 PLC 程序语义资产与现场 Adapter 摘要。"""

        try:
            qualification = self.programs[qualification_ref]
        except KeyError as exc:
            raise ValueError(f"缺少 ProgramQualification: {qualification_ref}") from exc
        _require_mode(qualification.qualification_ref, qualification.approved_for, mode)
        expected = (
            qualification.program_set_digest == program_set_digest,
            qualification.program_ref == program_ref,
            qualification.plc_adapter_digest == plc_adapter_digest,
        )
        if not all(expected):
            raise ValueError(f"{qualification_ref} 与当前 PLC 程序或 Adapter 不匹配")
        return qualification


def _validate_common(
    qualification_ref: str,
    evidence_ref: str,
    approved_by: str,
    approved_for: frozenset[DeploymentMode],
    digests: Iterable[str],
) -> None:
    """校验所有资格共享的身份、证据、模式与摘要。"""

    if any(
        not value.strip() for value in (qualification_ref, evidence_ref, approved_by)
    ):
        raise ValueError("Qualification identity、evidence 与 approved_by 不得为空")
    if not approved_for:
        raise ValueError("Qualification.approved_for 不得为空")
    invalid = [value for value in digests if _DIGEST.fullmatch(value.lower()) is None]
    if invalid:
        raise ValueError("Qualification 中所有 digest 必须是 64 位 SHA-256")


def _require_mode(
    qualification_ref: str,
    approved_for: frozenset[DeploymentMode],
    mode: DeploymentMode,
) -> None:
    """要求记录显式批准当前部署模式。"""

    if mode not in approved_for:
        raise ValueError(f"{qualification_ref} 未批准部署模式 {mode.value}")


def _record_index(values: Any, factory: Any) -> dict[str, Any]:
    """把资格数组转换为无重复稳定索引。"""

    if not isinstance(values, list):
        raise TypeError("Qualification 记录集合必须是列表")
    result: dict[str, Any] = {}
    for value in values:
        if not isinstance(value, Mapping):
            raise TypeError("Qualification 记录必须是对象")
        record = factory(value)
        if record.qualification_ref in result:
            raise ValueError(f"重复 qualification_ref: {record.qualification_ref}")
        result[record.qualification_ref] = record
    return result


def _point_record(value: Mapping[str, Any]) -> PointQualification:
    """解析一条点位资格记录。"""

    return PointQualification(
        str(value["qualification_ref"]),
        str(value["point_set_digest"]),
        str(value["target_ref"]),
        str(value["resolved_target_digest"]),
        str(value["arm_model_digest"]),
        str(value["tool_context_digest"]),
        str(value["installation_calibration_digest"]),
        str(value["collision_environment_digest"]),
        str(value["evidence_ref"]),
        str(value["approved_by"]),
        _modes(value.get("approved_for")),
    )


def _program_record(value: Mapping[str, Any]) -> ProgramQualification:
    """解析一条 PLC 程序资格记录。"""

    return ProgramQualification(
        str(value["qualification_ref"]),
        str(value["program_set_digest"]),
        str(value["program_ref"]),
        str(value["plc_adapter_digest"]),
        str(value["evidence_ref"]),
        str(value["approved_by"]),
        _modes(value.get("approved_for")),
    )


def _site_operation_record(value: Mapping[str, Any]) -> SiteOperationQualification:
    """解析一条 Site 端到端操作资格记录。"""

    return SiteOperationQualification(
        str(value["qualification_ref"]),
        str(value["declarations_digest"]),
        str(value["declaration_ref"]),
        ActionKind(str(value["action"])),
        _strings(value.get("payload_profiles"), "payload_profiles"),
        str(value["execution_qualification_ref"]),
        (
            None
            if value.get("motion_profiles_digest") is None
            else str(value["motion_profiles_digest"])
        ),
        str(value["occupancy_observation_ref"]),
        str(value["evidence_ref"]),
        str(value["approved_by"]),
        _modes(value.get("approved_for")),
    )


def _modes(value: Any) -> frozenset[DeploymentMode]:
    """解析资格批准的部署模式集合。"""

    if isinstance(value, (str, bytes)):
        raise TypeError("approved_for 必须是部署模式列表")
    try:
        return frozenset(DeploymentMode(str(item)) for item in value)
    except TypeError as exc:
        raise TypeError("approved_for 必须是部署模式列表") from exc


def _strings(value: Any, field: str) -> frozenset[str]:
    """解析非空字符串集合并拒绝把单个字符串误当数组。"""

    if isinstance(value, (str, bytes)):
        raise TypeError(f"{field} 必须是字符串列表")
    try:
        result = frozenset(str(item) for item in value)
    except TypeError as exc:
        raise TypeError(f"{field} 必须是字符串列表") from exc
    if not result or any(not item.strip() for item in result):
        raise ValueError(f"{field} 必须包含非空值")
    return result


__all__ = [
    "PointQualification",
    "ProgramQualification",
    "QualificationCatalog",
    "QualificationScope",
    "SiteOperationQualification",
]
