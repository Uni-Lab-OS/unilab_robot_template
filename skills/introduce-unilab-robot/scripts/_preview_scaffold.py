"""从 docs/demo 模板生成 Preview 领域设备骨架。"""

from __future__ import annotations

import re
from pathlib import Path


def _class_prefix(device_id: str) -> str:
    parts = [part for part in re.split(r"[_\-\s]+", device_id.strip()) if part]
    return "".join(part[:1].upper() + part[1:] for part in parts) or "PreviewArm"


def _render_template(text: str, *, domain_pkg: str, device_id: str, device_title: str) -> str:
    prefix = _class_prefix(device_id)
    mapping = {
        "{domain_pkg}": domain_pkg,
        "{device_id}": device_id,
        "{device_title}": device_title,
        "{class_prefix}": prefix,
    }
    rendered = text
    for key, value in mapping.items():
        rendered = rendered.replace(key, value)
    return rendered


def scaffold_preview_device(
    domain: Path,
    *,
    domain_pkg: str,
    device_id: str,
    device_title: str,
    demo_dir: Path,
    apply: bool,
) -> dict[str, object]:
    device_dir = domain / domain_pkg / "devices" / device_id
    card_file = f"{device_id}_arm_card.py"
    mapping = {
        "model.py": demo_dir / "model.py.template",
        "mounts.py": demo_dir / "mounts.py.template",
        "preview_kinematics.py": demo_dir / "preview_kinematics.py.template",
        "device.py": demo_dir / "device.py.template",
        card_file: demo_dir / "arm_card.py.template",
    }
    planned: list[str] = []
    created: list[str] = []
    for filename, template_path in mapping.items():
        if not template_path.is_file():
            raise FileNotFoundError(f"缺少 Preview demo 模板: {template_path}")
        target = device_dir / filename
        rel = str(target.relative_to(domain)).replace("\\", "/")
        if target.is_file():
            continue
        planned.append(rel)
        if apply:
            device_dir.mkdir(parents=True, exist_ok=True)
            content = _render_template(
                template_path.read_text(encoding="utf-8"),
                domain_pkg=domain_pkg,
                device_id=device_id,
                device_title=device_title,
            )
            target.write_text(content, encoding="utf-8", newline="\n")
            created.append(rel)
    return {
        "device_dir": str(device_dir.relative_to(domain)).replace("\\", "/"),
        "planned_files": planned,
        "created_files": created,
        "changed": bool(planned),
        "demo_reference": f"docs/demo/{demo_dir.name}",
    }
