#!/usr/bin/env python3
"""把 runtime manifest 的 arm 模块从 moveit_model 改指 robot_module。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from _domain_scan import (  # noqa: E402
    find_arm_device,
    patch_manifest_arm_module,
    read_manifest_arm_module,
    runtime_assembly_files,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="修复 runtime manifest arm 模块路径")
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

    patches: list[dict[str, str | bool | None]] = []
    for assembly_path in runtime_assembly_files(record.device_dir):
        original = assembly_path.read_text(encoding="utf-8")
        updated, changed = patch_manifest_arm_module(
            original,
            provider_module=record.provider_module,
            robot_module_path=record.robot_module_path,
        )
        rel = str(assembly_path.relative_to(record.domain)).replace("\\", "/")
        patches.append(
            {
                "path": rel,
                "changed": changed,
                "before": read_manifest_arm_module(original),
                "after": read_manifest_arm_module(updated),
            }
        )
        if changed and args.apply:
            assembly_path.write_text(updated, encoding="utf-8", newline="\n")

    needs_change = any(item["changed"] for item in patches)
    aligned = bool(patches) and all(
        item.get("after") == record.robot_module_path for item in patches
    )
    ok = aligned and (args.apply or not needs_change)
    print(
        json.dumps(
            {
                "ok": ok,
                "dry_run": not args.apply,
                "robot_module_path": record.robot_module_path,
                "patches": patches,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
