"""领域仓机械臂设备卡片 scaffold。"""

from __future__ import annotations

import json
from pathlib import Path


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
        "sdkVersion": "^0.1.0",
        "hostProtocolVersion": 1,
        "authoringProfile": "web-component-lite-v1",
        "entry": "src/index.ts",
        "uiFeatures": ["core", "manual-exclusive"],
        "permissions": {
            "state": ["actionBusy", "jointState", "moveit_online", "online"],
            "actions": [
                "home",
                "jog_joint_once",
                "jog_tcp_once",
                "move_rail_to_position",
                "move_to_anchor",
                "query",
                "read_debug_snapshot",
                "record_current_point",
                "teach_point_from_current",
            ],
            "media": [],
        },
        "config": {"version": 1, "defaults": {}, "schema": {}},
        "templateCard": "unilab_robot_template/frontend/cards/rail-mounted-arm-card",
    }
    branding = f"""/** {title} branding */
export const cardBranding = {{
  eyebrow: '{domain_pkg.upper()} · DEVICE COMMISSIONING',
  title: '{title}',
  subtitle: 'MoveIt · 可选导轨 · 本地 Edge Runtime'
}}
"""
    readme = f"""# {title}

Canonical 实现：[`unilab_robot_template/frontend/cards/rail-mounted-arm-card`](../../../../unilab_robot_template/frontend/cards/rail-mounted-arm-card)

后端 mixin：`unilab_robot_runtime.device_card.RailMountedArmCardMixin`
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
