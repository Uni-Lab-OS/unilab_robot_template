#!/usr/bin/env python3
"""检查关节命名：canonical=joint_N，qualified={device_id}_joint_N。"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from _domain_scan import find_arm_device, runtime_assembly_files  # noqa: E402
from _joint_naming import (  # noqa: E402
    expected_qualified_joint_names,
    find_legacy_qualified_joint_patterns,
    joint_naming_remediation,
    validate_canonical_joint_names,
)
from _model_facts import (  # noqa: E402
    read_arm_model_facts,
    read_moveit_model_joint_mapping,
    read_py_canonical_joint_names,
)

_ARM_ENDPOINT = re.compile(
    r'arm_endpoint\s*=\s*f"moveit:\{normalized_device\}:([a-z0-9]+)"'
)
_CATALOG_PY = re.compile(r"\bunilab_arm_[a-z0-9]+\b")
_SCAN_SUFFIXES = {".py", ".yaml", ".yml", ".json"}


def _scan_legacy_literals(*, root: Path, device_id: str) -> list[dict[str, str]]:
    hits: list[dict[str, str]] = []
    extra_suffixes = {".ts", ".tsx"}
    for path in root.rglob("*"):
        if not path.is_file() or ".git" in path.parts:
            continue
        if path.suffix.lower() not in _SCAN_SUFFIXES | extra_suffixes:
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for literal in find_legacy_qualified_joint_patterns(text, device_id=device_id):
            rel = str(path.relative_to(root)).replace("\\", "/")
            hits.append({"path": rel, "literal": literal})
    return hits


def check(domain: Path, device_id: str) -> dict[str, Any]:
    domain = domain.resolve()
    errors: list[str] = []
    warnings: list[str] = []

    record = find_arm_device(domain, device_id)
    if record is None:
        return {"ok": False, "errors": [f"未找到 package_moveit device: {device_id}"], "warnings": []}

    moveit_model_path = record.device_dir / "moveit_model.py"
    model_yaml_path = record.device_dir / "models" / "model.yaml"
    facts = read_arm_model_facts(
        moveit_model_path=moveit_model_path,
        domain_pkg=record.domain_pkg,
        device_id=record.device_id,
    )
    py_canonical = read_py_canonical_joint_names(moveit_model_path)
    mapped_joints = read_moveit_model_joint_mapping(moveit_model_path)

    if model_yaml_path.is_file():
        data = yaml.safe_load(model_yaml_path.read_text(encoding="utf-8")) or {}
        yaml_joints = tuple(str(value) for value in (data.get("kinematic_joints") or ()))
        if yaml_joints and py_canonical and yaml_joints != py_canonical:
            errors.append(
                "models/model.yaml kinematic_joints 与 moveit_model._CANONICAL_JOINT_NAMES 不一致: "
                f"yaml={yaml_joints} py={py_canonical}"
            )
        moveit_group = str(data.get("moveit_group") or "").strip()
        expected_group = facts.moveit_group
        if moveit_group and moveit_group != expected_group:
            errors.append(
                f"models/model.yaml moveit_group 必须是 {expected_group}，当前 {moveit_group!r}"
            )
    else:
        warnings.append("models/model.yaml 不存在，跳过 YAML 交叉校验")

    canonical = facts.canonical_joint_names
    dof = len(canonical) if canonical else 6
    errors.extend(validate_canonical_joint_names(canonical, dof=dof))

    if mapped_joints:
        mapped_set = set(mapped_joints)
        canonical_set = set(canonical)
        if mapped_set != canonical_set:
            errors.append(
                "moveit_model._JOINT_NAMES 映射值与 kinematic_joints 不一致: "
                f"mapping={tuple(sorted(mapped_set))} canonical={canonical}"
            )

    domain_legacy = _scan_legacy_literals(root=domain, device_id=record.device_id)
    for hit in domain_legacy:
        errors.append(
            f"{hit['path']} 仍含型号 slug qualified 关节: {hit['literal']!r}；"
            f"必须是 {record.device_id}_joint_<N>"
        )

    for assembly_path in runtime_assembly_files(record.device_dir):
        text = assembly_path.read_text(encoding="utf-8")
        rel = assembly_path.relative_to(domain)
        endpoint_match = _ARM_ENDPOINT.search(text)
        if endpoint_match and endpoint_match.group(1) != facts.arm_endpoint_suffix:
            errors.append(
                f"{rel} arm_endpoint 后缀必须是 {facts.arm_endpoint_suffix!r}，"
                f"当前 {endpoint_match.group(1)!r}"
            )
        if _CATALOG_PY.search(text):
            errors.append(f"{rel} 仍 import catalog 包 unilab_arm_*")

    remediation = joint_naming_remediation(device_id=record.device_id, dof=dof)

    return {
        "ok": not errors,
        "device_id": device_id,
        "arm_endpoint": facts.arm_endpoint,
        "moveit_group": facts.moveit_group,
        "canonical_joint_names": canonical,
        "expected_qualified_joint_names": expected_qualified_joint_names(
            device_id=device_id,
            dof=dof,
        ),
        "domain_legacy_qualified_literals": domain_legacy,
        "remediation": remediation,
        "errors": errors,
        "warnings": warnings,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="检查关节命名：canonical=joint_N，qualified={device_id}_joint_N"
    )
    parser.add_argument("--domain", required=True, type=Path)
    parser.add_argument("--device", required=True)
    args = parser.parse_args()
    result = check(args.domain, args.device)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
