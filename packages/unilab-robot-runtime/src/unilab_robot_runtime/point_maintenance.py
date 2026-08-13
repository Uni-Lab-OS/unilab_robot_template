"""PointSet 草稿校验、低速试运行、资格确认与不可变发布。"""

from __future__ import annotations

import hashlib
import json
import re
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import yaml
from unilab_robot_contracts import (
    CommandState,
    MotionTargetResolver,
    MoveTargetCommand,
    ResolvedMotionTarget,
    ToolContext,
)


@dataclass(frozen=True, slots=True)
class ValidatedPointSet:
    """已通过 exact 型号、工具和引用链校验的点位草稿快照。"""

    source_path: Path
    digest: str
    revision: str
    target_refs: tuple[str, ...]
    targets: dict[str, ResolvedMotionTarget]


@dataclass(frozen=True, slots=True)
class PointTestEvidence:
    """一个目标由统一调试端口低速完成的见证。"""

    point_set_digest: str
    target_ref: str
    command_id: str
    completed_at: float


@dataclass(frozen=True, slots=True)
class PointQualification:
    """对一个不可变草稿摘要的人工资格确认。"""

    point_set_digest: str
    revision: str
    target_refs: tuple[str, ...]
    evidence_command_ids: tuple[tuple[str, str], ...]
    approved_by: str
    approved_at: float
    qualification_id: str


@dataclass(frozen=True, slots=True)
class PublishedPointSet:
    """发布目录中的不可变 PointSet 制品。"""

    path: Path
    digest: str
    revision: str
    qualification_id: str


class PointMaintenanceService:
    """隐藏点位 YAML、试运行证据和不可变发布细节的深模块。"""

    def __init__(
        self,
        *,
        model: Any,
        tool_context: ToolContext,
        qualification_root: str | Path,
        publication_root: str | Path,
    ) -> None:
        """冻结 exact Arm/ToolContext，并配置本地资格与发布目录。"""

        self.model = model
        self.tool_context = tool_context
        self.qualification_root = Path(qualification_root)
        self.publication_root = Path(publication_root)
        self._test_evidence: dict[str, dict[str, PointTestEvidence]] = {}

    def validate_draft(self, source_path: str | Path) -> ValidatedPointSet:
        """解析整个草稿并返回绑定原始字节摘要的验证快照。"""

        path = Path(source_path)
        source = path.read_bytes()
        data = yaml.safe_load(source) or {}
        if not isinstance(data, dict):
            raise TypeError("PointSet 草稿必须是 YAML 对象")
        resolver = MotionTargetResolver(
            data,
            model=self.model,
            tool_context=self.tool_context,
        )
        targets = dict(resolver.resolve_all())
        return ValidatedPointSet(
            source_path=path,
            digest=hashlib.sha256(source).hexdigest(),
            revision=resolver.revision,
            target_refs=tuple(sorted(targets)),
            targets=targets,
        )

    def test_target(
        self,
        validated: ValidatedPointSet,
        target_ref: str,
        *,
        session: Any,
        command_id: str,
        hardware_profile_digest: str,
        source_boot_id: str,
        monotonic_sequence: int,
        motion_profile_ref: str = "maintenance-slow",
        velocity_scale: float = 0.05,
        acceleration_scale: float = 0.05,
    ) -> PointTestEvidence:
        """通过独占维护会话对活动草稿中的一个目标做低速试运行。"""

        self._require_unchanged(validated)
        if target_ref not in validated.targets:
            raise ValueError(f"PointSet 不包含 target_ref: {target_ref}")
        if getattr(session, "target_revision", None) != validated.revision:
            raise RuntimeError(
                "维护运行时未激活该草稿 revision；必须使用绑定候选 PointSet 的维护 manifest"
            )
        if velocity_scale > 0.1 or acceleration_scale > 0.1:
            raise ValueError("点位资格试运行速度和加速度不得超过 10%")
        command = MoveTargetCommand(
            command_id=command_id,
            hardware_profile_digest=hardware_profile_digest,
            source_boot_id=source_boot_id,
            monotonic_sequence=monotonic_sequence,
            motion_profile_ref=motion_profile_ref,
            velocity_scale=velocity_scale,
            acceleration_scale=acceleration_scale,
            target_ref=target_ref,
            target_revision=validated.revision,
        )
        result = session.execute(command)
        if result.state is not CommandState.SUCCEEDED:
            raise RuntimeError(
                f"点位 {target_ref} 低速试运行未完成: {result.state.value}"
            )
        evidence = PointTestEvidence(
            validated.digest,
            target_ref,
            command_id,
            time.time(),
        )
        self._test_evidence.setdefault(validated.digest, {})[target_ref] = evidence
        evidence_path = (
            self.qualification_root
            / "evidence"
            / validated.digest
            / f"{_slug(target_ref)}.json"
        )
        evidence_path.parent.mkdir(parents=True, exist_ok=True)
        _write_immutable_json(evidence_path, asdict(evidence))
        return evidence

    def qualify(
        self,
        validated: ValidatedPointSet,
        *,
        approved_by: str,
    ) -> PointQualification:
        """要求全部目标具有成功试运行见证后生成资格确认记录。"""

        self._require_unchanged(validated)
        approver = str(approved_by).strip()
        if not approver:
            raise ValueError("点位资格确认必须包含 approved_by")
        evidence = self._test_evidence.get(validated.digest, {})
        missing = tuple(
            target_ref
            for target_ref in validated.target_refs
            if target_ref not in evidence
        )
        if missing:
            raise RuntimeError("尚有目标未完成低速试运行: " + ", ".join(missing))
        approved_at = time.time()
        qualification_id = hashlib.sha256(
            f"{validated.digest}:{approver}:{approved_at:.9f}".encode()
        ).hexdigest()
        qualification = PointQualification(
            validated.digest,
            validated.revision,
            validated.target_refs,
            tuple(
                (target_ref, evidence[target_ref].command_id)
                for target_ref in validated.target_refs
            ),
            approver,
            approved_at,
            qualification_id,
        )
        self.qualification_root.mkdir(parents=True, exist_ok=True)
        path = self.qualification_root / f"{validated.digest}.json"
        _write_immutable_json(path, asdict(qualification))
        return qualification

    def publish(
        self,
        validated: ValidatedPointSet,
        qualification: PointQualification,
    ) -> PublishedPointSet:
        """把与资格记录相同摘要的草稿发布为不可变文件。"""

        self._require_unchanged(validated)
        if (
            qualification.point_set_digest != validated.digest
            or qualification.revision != validated.revision
            or qualification.target_refs != validated.target_refs
        ):
            raise ValueError("PointQualification 与验证草稿不一致")
        qualification_path = self.qualification_root / f"{validated.digest}.json"
        if not qualification_path.is_file():
            raise ValueError("资格确认记录尚未持久化")
        self.publication_root.mkdir(parents=True, exist_ok=True)
        revision_slug = _slug(validated.revision)
        destination = self.publication_root / (
            f"{revision_slug}.{validated.digest[:12]}.yaml"
        )
        source = validated.source_path.read_bytes()
        if destination.exists():
            if destination.read_bytes() != source:
                raise RuntimeError("不可变发布路径已存在不同内容")
        else:
            temporary = destination.with_suffix(destination.suffix + ".tmp")
            temporary.write_bytes(source)
            temporary.replace(destination)
        return PublishedPointSet(
            destination,
            validated.digest,
            validated.revision,
            qualification.qualification_id,
        )

    @staticmethod
    def _require_unchanged(validated: ValidatedPointSet) -> None:
        """确保草稿字节仍与验证摘要完全一致。"""

        current = hashlib.sha256(validated.source_path.read_bytes()).hexdigest()
        if current != validated.digest:
            raise RuntimeError("PointSet 草稿在验证后发生变化，必须重新校验")


def _write_immutable_json(path: Path, payload: dict[str, Any]) -> None:
    """写入稳定 JSON；同一路径只接受完全相同的内容。"""

    encoded = json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    if path.exists():
        if path.read_bytes() != encoded:
            raise RuntimeError("资格确认记录已存在不同内容")
        return
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_bytes(encoded)
    temporary.replace(path)


def _slug(value: str) -> str:
    """把稳定引用规范化为可移植文件名片段。"""

    return re.sub(r"[^A-Za-z0-9_.-]+", "-", value)


__all__ = [
    "PointMaintenanceService",
    "PointQualification",
    "PointTestEvidence",
    "PublishedPointSet",
    "ValidatedPointSet",
]
