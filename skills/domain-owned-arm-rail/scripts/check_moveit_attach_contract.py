#!/usr/bin/env python3
"""检查型号无关的 MoveIt 工具附着合同。"""

from __future__ import annotations

import argparse
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from _domain_scan import (  # noqa: E402
    find_arm_device,
    read_manifest_arm_module,
    runtime_assembly_files,
)
from _model_facts import (  # noqa: E402
    expected_domain_model_ref,
    read_arm_model_facts,
    read_moveit_model_descriptor_ref,
)

_LITERAL_LINK = re.compile(r"""['"][A-Za-z0-9_]+_[A-Za-z0-9_]+_tool0['"]""")
_FLANGE = re.compile(r"""flange_frame\s*=\s*['"]([^'"]+)['"]""")
_CATALOG_PY = re.compile(r"\bunilab_arm_[a-z0-9]+\b")
_CATALOG_DIST = re.compile(r'"unilab-arm-[a-z0-9\-]+"')
_ARM_ENDPOINT = re.compile(
    r'arm_endpoint\s*=\s*f"moveit:\{normalized_device\}:([a-z0-9]+)"'
)
_COMMISSIONING_FILES = (
    "moveit_commissioning.py",
    "robot_commissioning.py",
)


def check(domain: Path, device_id: str) -> dict[str, Any]:
    domain = domain.resolve()
    errors: list[str] = []
    warnings: list[str] = []
    record = find_arm_device(domain, device_id)
    if record is None:
        return {"ok": False, "errors": [f"未找到 package_moveit device: {device_id}"], "warnings": []}

    model_files = sorted(record.device_dir.glob("models/*.urdf"))
    model_path = record.device_dir / "moveit_model.py"
    expected_flange = ""
    if not model_files:
        errors.append("models/ 下没有 vendor URDF")
    else:
        ET.parse(model_files[0])
        match = _FLANGE.search(model_path.read_text(encoding="utf-8")) if model_path.is_file() else None
        if not match:
            errors.append("moveit_model.py 未声明来自当前 URDF 的 flange_frame")
        else:
            expected_flange = f"{device_id}_{match.group(1)}"

    asset_roots = [
        domain / "deployment" / "robot_cell" / "assets",
        domain / record.domain_pkg / "deployment" / "robot_cell" / "assets",
    ]
    asset_root = next((root for root in asset_roots if root.is_dir()), asset_roots[0])
    context_files = sorted((asset_root / "tool_contexts").glob("*.y*ml"))
    expected_model_ref = expected_domain_model_ref(record.domain_pkg, device_id)
    current_model_ref = read_moveit_model_descriptor_ref(model_path)
    if current_model_ref is None:
        errors.append("moveit_model.py 未声明 MODEL_DESCRIPTOR.model_ref")
    elif current_model_ref != expected_model_ref:
        errors.append(
            "moveit_model.py MODEL_DESCRIPTOR.model_ref 必须与 PointSet 一致: "
            f"期望 {expected_model_ref}，当前 {current_model_ref}"
        )
    for path in sorted((asset_root / "point_sets").glob("*.y*ml")):
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        arm = ((data.get("components") or {}).get("arm") or {}) if isinstance(data, dict) else {}
        if isinstance(arm, dict) and "model_ref" in arm and arm.get("model_ref") != expected_model_ref:
            errors.append(
                f"{path.relative_to(domain)} components.arm.model_ref 必须是 {expected_model_ref}"
            )
    for path in context_files:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if not isinstance(data, dict):
            errors.append(f"{path.relative_to(domain)} 不是对象")
            continue
        if "mount_link" in data or data.get("mount_link_ref") != "moveit.end_effector":
            errors.append(f"{path.relative_to(domain)} 必须使用 mount_link_ref=moveit.end_effector")
        scene = data.get("planning_scene") or {}
        if not isinstance(scene, dict):
            errors.append(f"{path.relative_to(domain)} planning_scene 不是对象")
            continue
        if "parent_link" in scene or scene.get("parent_link_ref") != "moveit.end_effector":
            errors.append(f"{path.relative_to(domain)} 必须使用 parent_link_ref=moveit.end_effector")
        refs = scene.get("allowed_touch_link_refs")
        if not isinstance(refs, list) or "moveit.end_effector" not in refs:
            errors.append(f"{path.relative_to(domain)} allowed_touch_link_refs 未包含 moveit.end_effector")
        collision_asset = str(scene.get("collision_asset_ref") or "")
        if not collision_asset.lower().endswith(".stl"):
            errors.append(f"{path.relative_to(domain)} collision_asset_ref 必须是可替换 STL")

    facts = read_arm_model_facts(
        moveit_model_path=model_path,
        domain_pkg=record.domain_pkg,
        device_id=record.device_id,
    )
    expected_endpoint = facts.arm_endpoint

    for assembly_path in runtime_assembly_files(record.device_dir):
        text = assembly_path.read_text(encoding="utf-8")
        rel = assembly_path.relative_to(domain)
        arm_module = read_manifest_arm_module(text)
        if arm_module != record.robot_module_path:
            errors.append(
                f"{rel} manifest arm.python_package 必须是 {record.robot_module_path}，当前 {arm_module!r}"
            )
        if _CATALOG_PY.search(text):
            errors.append(f"{rel} 仍 import catalog 包 unilab_arm_*")
        if _CATALOG_DIST.search(text):
            errors.append(f"{rel} 仍引用 catalog distribution unilab-arm-*")
        endpoint_match = _ARM_ENDPOINT.search(text)
        if endpoint_match and endpoint_match.group(1) != facts.arm_endpoint_suffix:
            errors.append(
                f"{rel} arm_endpoint 后缀必须是 {facts.arm_endpoint_suffix!r}，"
                f"当前 {endpoint_match.group(1)!r}"
            )

    for name in _COMMISSIONING_FILES:
        path = record.device_dir / name
        if not path.is_file():
            continue
        text = path.read_text(encoding="utf-8")
        rel = path.relative_to(domain)
        if _CATALOG_PY.search(text):
            errors.append(f"{rel} 仍 import catalog 包 unilab_arm_*")
        commissioning_endpoint_ok = (
            facts.arm_endpoint in text
            or re.search(
                r'moveit:\{[^}]+\}:arm|"moveit:\{[^}]+\}:arm"',
                text,
            )
            is not None
        )
        if not commissioning_endpoint_ok:
            errors.append(
                f"{rel} commissioning endpoint 未指向 moveit:{{device_id}}:arm"
            )

    if expected_flange:
        scan_roots = [
            domain / "szlab_poly_studio" / "robot_cell",
            asset_root,
            record.device_dir / "config",
        ]
        for root in scan_roots:
            if not root.is_dir():
                continue
            for path in root.rglob("*"):
                if not path.is_file() or "tests" in path.parts or path.suffix.lower() not in {".py", ".yaml", ".yml", ".json"}:
                    continue
                text = path.read_text(encoding="utf-8", errors="ignore")
                for match in _LITERAL_LINK.finditer(text):
                    literal = match.group(0).strip("'\"")
                    if literal != expected_flange:
                        errors.append(f"{path.relative_to(domain)} 含型号限定 attach link: {literal}")

    if not context_files:
        warnings.append("deployment/robot_cell/assets/tool_contexts 下没有 ToolContext")
    return {
        "ok": not errors,
        "device_id": device_id,
        "expected_model_ref": expected_model_ref,
        "current_model_ref": current_model_ref,
        "expected_end_effector": expected_flange,
        "errors": errors,
        "warnings": warnings,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="检查 MoveIt attach 是否使用当前机械臂动态末端引用")
    parser.add_argument("--domain", required=True, type=Path)
    parser.add_argument("--device", required=True)
    args = parser.parse_args()
    result = check(args.domain, args.device)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
