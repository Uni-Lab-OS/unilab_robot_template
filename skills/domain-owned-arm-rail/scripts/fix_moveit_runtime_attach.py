#!/usr/bin/env python3
"""把 MoveIt attach 执行链从 catalog 型号改绑到领域 robot_module。"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from _domain_scan import (  # noqa: E402
    find_arm_device,
    patch_manifest_arm_module,
    read_manifest_arm_module,
    runtime_assembly_files,
)
from _model_facts import (  # noqa: E402
    expected_domain_model_ref,
    patch_moveit_model_descriptor_ref,
    read_arm_model_facts,
    read_moveit_model_descriptor_ref,
)

_CATALOG_PY = re.compile(r"\bunilab_arm_[a-z0-9]+\b")
_CATALOG_DIST = re.compile(r'"unilab-arm-[a-z0-9\-]+"')
_CATALOG_MODEL_REF = re.compile(r"package://unilab_arm_[a-z0-9]+/models/model\.yaml")
_ARM_ENDPOINT = re.compile(
    r'(arm_endpoint\s*=\s*f"moveit:\{normalized_device\}:)([a-z0-9]+)(")'
)
_COMMISSIONING_ENDPOINT = re.compile(
    r'(frozenset\(\{f"moveit:\{[^}]+\}:)([a-z0-9]+)(\}"\}\))'
)
_COMMISSIONING_FILES = (
    "moveit_commissioning.py",
    "robot_commissioning.py",
)


def _replace_catalog_imports(text: str, *, robot_module_path: str) -> tuple[str, int]:
    replacements = 0

    def repl(match: re.Match[str]) -> str:
        nonlocal replacements
        replacements += 1
        return robot_module_path

    return _CATALOG_PY.sub(repl, text), replacements


def _replace_endpoint_suffixes(text: str, *, endpoint_suffix: str) -> tuple[str, int]:
    replacements = 0

    def repl_arm(match: re.Match[str]) -> str:
        nonlocal replacements
        if match.group(2) == endpoint_suffix:
            return match.group(0)
        replacements += 1
        return f"{match.group(1)}{endpoint_suffix}{match.group(3)}"

    def repl_commissioning(match: re.Match[str]) -> str:
        nonlocal replacements
        if match.group(2) == endpoint_suffix:
            return match.group(0)
        replacements += 1
        return f"{match.group(1)}{endpoint_suffix}{match.group(3)}"

    updated = _ARM_ENDPOINT.sub(repl_arm, text)
    updated = _COMMISSIONING_ENDPOINT.sub(repl_commissioning, updated)
    return updated, replacements


def patch_runtime_assembly(
    text: str,
    *,
    endpoint_suffix: str,
    distribution: str,
    robot_module_path: str,
    provider_module: str,
) -> tuple[str, dict[str, int]]:
    counts = {"catalog_import": 0, "catalog_dist": 0, "endpoint": 0, "manifest": 0}
    updated, count = _replace_catalog_imports(text, robot_module_path=robot_module_path)
    counts["catalog_import"] += count
    updated, count = _replace_endpoint_suffixes(updated, endpoint_suffix=endpoint_suffix)
    counts["endpoint"] += count
    if _CATALOG_DIST.search(updated):
        updated = _CATALOG_DIST.sub(f'"{distribution}"', updated)
        counts["catalog_dist"] += 1
    updated, changed = patch_manifest_arm_module(
        updated,
        provider_module=provider_module,
        robot_module_path=robot_module_path,
    )
    if changed:
        counts["manifest"] += 1
    return updated, counts


def patch_commissioning_file(
    text: str,
    *,
    endpoint_suffix: str,
    robot_module_path: str,
) -> tuple[str, dict[str, int]]:
    counts = {"catalog_import": 0, "endpoint": 0}
    updated, count = _replace_catalog_imports(text, robot_module_path=robot_module_path)
    counts["catalog_import"] += count
    updated, count = _replace_endpoint_suffixes(updated, endpoint_suffix=endpoint_suffix)
    counts["endpoint"] += count
    return updated, counts


def collect_patches(record: Any, facts: Any) -> list[dict[str, Any]]:
    patches: list[dict[str, Any]] = []
    robot_module = record.robot_module_path

    for assembly_path in runtime_assembly_files(record.device_dir):
        original = assembly_path.read_text(encoding="utf-8")
        updated, counts = patch_runtime_assembly(
            original,
            endpoint_suffix=facts.arm_endpoint_suffix,
            distribution=facts.distribution,
            robot_module_path=robot_module,
            provider_module=record.provider_module,
        )
        rel = str(assembly_path.relative_to(record.domain)).replace("\\", "/")
        patches.append(
            {
                "path": rel,
                "kind": "runtime_assembly",
                "changed": updated != original,
                "counts": counts,
                "before_arm_module": read_manifest_arm_module(original),
                "after_arm_module": read_manifest_arm_module(updated),
                "updated": updated,
            }
        )

    for name in _COMMISSIONING_FILES:
        path = record.device_dir / name
        if not path.is_file():
            continue
        original = path.read_text(encoding="utf-8")
        updated, counts = patch_commissioning_file(
            original,
            endpoint_suffix=facts.arm_endpoint_suffix,
            robot_module_path=robot_module,
        )
        rel = str(path.relative_to(record.domain)).replace("\\", "/")
        patches.append(
            {
                "path": rel,
                "kind": "commissioning",
                "changed": updated != original,
                "counts": counts,
                "updated": updated,
            }
        )
    return patches


def collect_moveit_model_patch(record: Any) -> dict[str, Any] | None:
    moveit_model_path = record.device_dir / "moveit_model.py"
    if not moveit_model_path.is_file():
        return None
    expected_model_ref = expected_domain_model_ref(
        record.domain_pkg,
        record.device_id,
    )
    original = moveit_model_path.read_text(encoding="utf-8")
    try:
        updated, changed = patch_moveit_model_descriptor_ref(original, expected_model_ref)
    except ValueError as exc:
        return {
            "path": str(moveit_model_path.relative_to(record.domain)).replace("\\", "/"),
            "kind": "moveit_model",
            "changed": False,
            "error": str(exc),
            "expected_model_ref": expected_model_ref,
            "current_model_ref": read_moveit_model_descriptor_ref(moveit_model_path),
            "updated": original,
        }
    return {
        "path": str(moveit_model_path.relative_to(record.domain)).replace("\\", "/"),
        "kind": "moveit_model",
        "changed": changed,
        "expected_model_ref": expected_model_ref,
        "before_model_ref": read_moveit_model_descriptor_ref(moveit_model_path),
        "after_model_ref": expected_model_ref if changed else read_moveit_model_descriptor_ref(moveit_model_path),
        "updated": updated,
    }


def collect_point_set_patches(record: Any) -> list[dict[str, Any]]:
    expected_model_ref = expected_domain_model_ref(
        record.domain_pkg,
        record.device_id,
    )
    asset_roots = [
        record.domain / "deployment" / "robot_cell" / "assets",
        record.domain / record.domain_pkg / "deployment" / "robot_cell" / "assets",
    ]
    patches: list[dict[str, Any]] = []
    seen: set[Path] = set()
    for root in asset_roots:
        point_sets_dir = root / "point_sets"
        if not point_sets_dir.is_dir():
            continue
        for path in sorted(point_sets_dir.glob("*.y*ml")):
            if path in seen:
                continue
            seen.add(path)
            original = path.read_text(encoding="utf-8")
            if _CATALOG_MODEL_REF.search(original) is None:
                continue
            updated = _CATALOG_MODEL_REF.sub(expected_model_ref, original)
            rel = str(path.relative_to(record.domain)).replace("\\", "/")
            patches.append(
                {
                    "path": rel,
                    "kind": "point_set",
                    "changed": updated != original,
                    "expected_model_ref": expected_model_ref,
                    "updated": updated,
                }
            )
    return patches


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="修复 MoveIt attach 执行链：catalog 型号 → 领域 robot_module"
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

    facts = read_arm_model_facts(
        moveit_model_path=record.device_dir / "moveit_model.py",
        domain_pkg=record.domain_pkg,
        device_id=record.device_id,
    )
    patches = collect_patches(record, facts)
    moveit_model_patch = collect_moveit_model_patch(record)
    point_set_patches = collect_point_set_patches(record)
    all_patches = [*patches]
    if moveit_model_patch is not None:
        all_patches.append(moveit_model_patch)
    all_patches.extend(point_set_patches)
    needs_change = any(item.get("changed") for item in all_patches)
    expected_model_ref = expected_domain_model_ref(
        record.domain_pkg,
        record.device_id,
    )
    current_model_ref = read_moveit_model_descriptor_ref(record.device_dir / "moveit_model.py")
    model_ref_aligned = current_model_ref == expected_model_ref
    aligned = bool(patches) and all(
        not item.get("after_arm_module")
        or item.get("after_arm_module") == record.robot_module_path
        for item in patches
        if item["kind"] == "runtime_assembly"
    )
    moveit_model_ok = moveit_model_patch is None or not moveit_model_patch.get("error")

    if args.apply:
        for patch in all_patches:
            if not patch["changed"]:
                continue
            target = record.domain / patch["path"]
            target.write_text(patch["updated"], encoding="utf-8", newline="\n")

    if args.apply and moveit_model_patch is not None and moveit_model_patch.get("changed"):
        current_model_ref = expected_model_ref
        model_ref_aligned = True

    ok = (
        aligned
        and model_ref_aligned
        and moveit_model_ok
        and (args.apply or not needs_change)
    )
    public_patches = [
        {key: value for key, value in patch.items() if key != "updated"}
        for patch in all_patches
    ]
    print(
        json.dumps(
            {
                "ok": ok,
                "dry_run": not args.apply,
                "arm_endpoint": facts.arm_endpoint,
                "moveit_group": facts.moveit_group,
                "expected_model_ref": expected_model_ref,
                "current_model_ref": current_model_ref,
                "model_ref_aligned": model_ref_aligned,
                "expected_end_effector": f"{record.device_id}_{facts.flange_frame}",
                "robot_module_path": record.robot_module_path,
                "distribution": facts.distribution,
                "patches": public_patches,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
