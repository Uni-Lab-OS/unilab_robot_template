#!/usr/bin/env python3
"""把 device 目录内关节名统一为 joint_N / {device_id}_joint_N，剔除型号 slug。"""

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

from _domain_scan import find_arm_device  # noqa: E402
from _joint_naming import (  # noqa: E402
    expected_canonical_joint_names,
    expected_qualified_joint_names,
    find_legacy_qualified_joint_patterns,
    joint_naming_remediation,
)

_SUFFIXES = {".py", ".yaml", ".yml", ".json"}
# canonical 含 slug 前缀：任意 xxx_joint_N，且 xxx != joint
_SLUG_CANONICAL = re.compile(r"\b(?!joint_)([a-z0-9]+)_joint_([1-9]\d*)\b", re.IGNORECASE)


def _qualified_slug_pattern(device_id: str) -> re.Pattern[str]:
    escaped = re.escape(device_id)
    return re.compile(rf"\b{escaped}_[a-z0-9]+_joint_([1-9]\d*)\b")


def _normalize_joint_text(text: str, *, device_id: str) -> tuple[str, int]:
    """去掉 canonical/qualified 关节名中的型号 slug。"""

    changes = 0
    updated = text

    def repl_qualified(match: re.Match[str]) -> str:
        nonlocal changes
        changes += 1
        return f"{device_id}_joint_{match.group(1)}"

    updated = _qualified_slug_pattern(device_id).sub(repl_qualified, updated)

    def repl_canonical(match: re.Match[str]) -> str:
        nonlocal changes
        prefix = match.group(1)
        index = match.group(2)
        if prefix == "joint":
            return match.group(0)
        changes += 1
        return f"joint_{index}"

    updated = _SLUG_CANONICAL.sub(repl_canonical, updated)
    return updated, changes


def _patch_model_yaml(path: Path, *, dof: int) -> tuple[str, bool]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} 不是 YAML 对象")
    canonical = list(expected_canonical_joint_names(dof=dof))
    changed = False
    if list(data.get("kinematic_joints") or ()) != canonical:
        data["kinematic_joints"] = canonical
        changed = True
    sources = data.get("joint_state_sources")
    if isinstance(sources, dict):
        if list(sources.get("canonical") or ()) != canonical:
            sources["canonical"] = canonical
            changed = True
    if not changed:
        return path.read_text(encoding="utf-8"), False
    return yaml.safe_dump(data, allow_unicode=True, sort_keys=False), True


def collect_patches(record: Any, *, dof: int = 6) -> list[dict[str, Any]]:
    device_dir = record.device_dir
    patches: list[dict[str, Any]] = []

    model_yaml = device_dir / "models" / "model.yaml"
    if model_yaml.is_file():
        original = model_yaml.read_text(encoding="utf-8")
        updated, yaml_changed = _patch_model_yaml(model_yaml, dof=dof)
        if not yaml_changed:
            updated, count = _normalize_joint_text(original, device_id=record.device_id)
            yaml_changed = count > 0
        rel = str(model_yaml.relative_to(record.domain)).replace("\\", "/")
        patches.append(
            {
                "path": rel,
                "kind": "model_yaml",
                "changed": yaml_changed and updated != original,
                "updated": updated if yaml_changed else original,
            }
        )

    for path in sorted(device_dir.rglob("*")):
        if not path.is_file() or "tests" in path.parts:
            continue
        if path.suffix.lower() not in _SUFFIXES:
            continue
        if path == model_yaml:
            continue
        original = path.read_text(encoding="utf-8")
        updated, count = _normalize_joint_text(original, device_id=record.device_id)
        if count == 0 and updated == original:
            continue
        rel = str(path.relative_to(record.domain)).replace("\\", "/")
        patches.append(
            {
                "path": rel,
                "kind": "text",
                "changed": updated != original,
                "replacements": count,
                "updated": updated,
            }
        )
    return patches


def scan_domain_legacy(*, domain: Path, device_id: str) -> list[dict[str, str]]:
    hits: list[dict[str, str]] = []
    for path in domain.rglob("*"):
        if not path.is_file() or ".git" in path.parts:
            continue
        if path.suffix.lower() not in _SUFFIXES and path.suffix.lower() not in {".ts", ".tsx"}:
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for literal in find_legacy_qualified_joint_patterns(text, device_id=device_id):
            rel = str(path.relative_to(domain)).replace("\\", "/")
            hits.append({"path": rel, "literal": literal})
    return hits


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="剔除关节名中的型号 slug，统一为 joint_N / {device_id}_joint_N"
    )
    parser.add_argument("--domain", required=True, type=Path)
    parser.add_argument("--device", required=True)
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args(argv)

    record = find_arm_device(args.domain.resolve(), args.device)
    if record is None:
        print(
            json.dumps(
                {"ok": False, "error": f"未找到 package_moveit device: {args.device}"},
                ensure_ascii=False,
            )
        )
        return 2

    patches = collect_patches(record)
    domain_legacy = scan_domain_legacy(domain=record.domain, device_id=record.device_id)
    needs_change = any(item["changed"] for item in patches)

    if args.apply:
        for patch in patches:
            if not patch["changed"]:
                continue
            target = record.domain / patch["path"]
            target.write_text(patch["updated"], encoding="utf-8", newline="\n")

    ok = (args.apply or not needs_change) and not domain_legacy
    print(
        json.dumps(
            {
                "ok": ok,
                "dry_run": not args.apply,
                "device_id": record.device_id,
                "expected_qualified_joint_names": expected_qualified_joint_names(
                    device_id=record.device_id
                ),
                "remediation": joint_naming_remediation(device_id=record.device_id),
                "patches": [
                    {k: v for k, v in patch.items() if k != "updated"}
                    for patch in patches
                ],
                "domain_legacy_qualified_literals": domain_legacy,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
