"""领域仓机械臂设备卡片 scaffold。"""

from __future__ import annotations

import json
import re
from pathlib import Path


def _class_prefix(device_id: str) -> str:
    parts = [part for part in re.split(r"[_\-\s]+", device_id.strip()) if part]
    return "".join(part[:1].upper() + part[1:] for part in parts) or "Arm"


def _render_template(text: str, *, domain_pkg: str, device_id: str, class_prefix: str) -> str:
    mapping = {
        "{domain_pkg}": domain_pkg,
        "{device_id}": device_id,
        "{class_prefix}": class_prefix,
    }
    rendered = text
    for key, value in mapping.items():
        rendered = rendered.replace(key, value)
    return rendered


def card_manifest_path(domain: Path, device_id: str) -> Path:
    card_dir = domain / "frontend" / "cards" / f"{device_id.replace('_', '-')}-card"
    return card_dir / "card.manifest.json"


def moveit_arm_card_path(domain: Path, domain_pkg: str, device_id: str) -> Path:
    return domain / domain_pkg / "devices" / device_id / f"{device_id}_arm_card.py"


def device_has_card_mixin(device_py_text: str, device_id: str) -> bool:
    return f"{_class_prefix(device_id)}ArmCardMixin" in device_py_text


def assess_arm_card(
    domain: Path,
    *,
    domain_pkg: str,
    device_id: str,
    device_py: Path | None,
    preview_mode: bool,
) -> dict[str, bool | list[str]]:
    missing: list[str] = []
    manifest = card_manifest_path(domain, device_id)
    if not manifest.is_file():
        missing.append(str(manifest.relative_to(domain)).replace("\\", "/"))
    if not preview_mode:
        arm_card = moveit_arm_card_path(domain, domain_pkg, device_id)
        if not arm_card.is_file():
            missing.append(str(arm_card.relative_to(domain)).replace("\\", "/"))
        if device_py is not None and device_py.is_file():
            if not device_has_card_mixin(device_py.read_text(encoding="utf-8"), device_id):
                missing.append(
                    str(device_py.relative_to(domain)).replace("\\", "/") + "#ArmCardMixin"
                )
        elif device_py is not None:
            missing.append(str(device_py.relative_to(domain)).replace("\\", "/"))
    return {
        "complete": not missing,
        "missing": missing,
        "manifest_exists": manifest.is_file(),
    }


def scaffold_moveit_arm_card(
    domain: Path,
    *,
    domain_pkg: str,
    device_id: str,
    demo_dir: Path,
    apply: bool = False,
) -> dict[str, str | bool | list[str]]:
    template_path = demo_dir / "arm_card.py.template"
    if not template_path.is_file():
        raise FileNotFoundError(f"缺少 MoveIt 卡片 demo 模板: {template_path}")
    class_prefix = _class_prefix(device_id)
    card_filename = f"{device_id}_arm_card.py"
    target = domain / domain_pkg / "devices" / device_id / card_filename
    rel = str(target.relative_to(domain)).replace("\\", "/")
    planned: list[str] = []
    created: list[str] = []
    if not target.is_file():
        planned.append(rel)
        if apply:
            target.parent.mkdir(parents=True, exist_ok=True)
            content = _render_template(
                template_path.read_text(encoding="utf-8"),
                domain_pkg=domain_pkg,
                device_id=device_id,
                class_prefix=class_prefix,
            )
            target.write_text(content, encoding="utf-8", newline="\n")
            created.append(rel)
    return {
        "arm_card_path": rel,
        "class_prefix": class_prefix,
        "planned_files": planned,
        "created_files": created,
        "changed": bool(planned),
        "demo_reference": f"docs/demo/{demo_dir.name}",
    }


def scaffold_domain_card(
    domain: Path,
    *,
    domain_pkg: str,
    device_id: str,
    title: str,
    apply: bool = False,
) -> dict[str, str | bool | list[str]]:
    device_type = f"community.{domain_pkg}.{device_id}"
    card_dir = domain / "frontend" / "cards" / f"{device_id.replace('_', '-')}-card"
    manifest_path = card_dir / "card.manifest.json"
    branding_path = card_dir / "src" / "branding.ts"
    readme_path = card_dir / "README.md"
    created: list[str] = []

    manifest = {
        "schemaVersion": 1,
        "id": f"{device_type}.card",
        "version": "0.1.0",
        "title": title,
        "deviceTypes": [device_type],
        "hostProtocolVersion": 1,
        "templateCard": "unilab_robot_template/frontend/cards/rail-mounted-arm-card",
    }
    branding = f"""/** {title} branding */
export const cardBranding = {{
  eyebrow: '{domain_pkg.upper()} · DEVICE COMMISSIONING',
  title: '{title}',
  subtitle: 'Preview / MoveIt 自适应 · 本地 Edge Runtime'
}}
"""
    readme = f"""# {title}

Canonical 实现：[`unilab_robot_template/frontend/cards/rail-mounted-arm-card`](../../../../unilab_robot_template/frontend/cards/rail-mounted-arm-card)

一张 template 卡按 Graph 设备自动切换 Preview 或 MoveIt UI；MoveIt 设备后端 mixin：`unilab_robot_runtime.device_card.RailMountedArmCardMixin`
"""

    planned: list[str] = []
    if not manifest_path.exists():
        planned.append(str(manifest_path.relative_to(domain)).replace("\\", "/"))
    if not branding_path.exists():
        planned.append(str(branding_path.relative_to(domain)).replace("\\", "/"))
    if not readme_path.exists():
        planned.append(str(readme_path.relative_to(domain)).replace("\\", "/"))
    if apply:
        if not manifest_path.exists():
            manifest_path.parent.mkdir(parents=True, exist_ok=True)
            manifest_path.write_text(
                json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            created.append(str(manifest_path.relative_to(domain)).replace("\\", "/"))
        if not branding_path.exists():
            branding_path.parent.mkdir(parents=True, exist_ok=True)
            branding_path.write_text(branding, encoding="utf-8", newline="\n")
            created.append(str(branding_path.relative_to(domain)).replace("\\", "/"))
        if not readme_path.exists():
            readme_path.write_text(readme, encoding="utf-8", newline="\n")
            created.append(str(readme_path.relative_to(domain)).replace("\\", "/"))

    return {
        "card_dir": str(card_dir.relative_to(domain)).replace("\\", "/"),
        "manifest_path": str(manifest_path.relative_to(domain)).replace("\\", "/"),
        "planned_files": planned,
        "created_files": created,
        "changed": bool(planned),
    }
