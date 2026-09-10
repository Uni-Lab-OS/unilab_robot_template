#!/usr/bin/env python3
"""生成领域机械臂 robot_module 相关文件。"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from _domain_scan import (  # noqa: E402
    find_arm_device,
    read_manifest_distribution_version,
    runtime_assembly_files,
)
from _templates import (  # noqa: E402
    adapters_init_py,
    arm_module_py,
    robot_factory_py,
    robot_module_py,
)


@dataclass
class WritePlan:
    path: Path
    content: str
    reason: str


def build_write_plan(
    *,
    device_dir: Path,
    domain_pkg: str,
    robot_device: str,
    module_version: str,
    force: bool,
) -> tuple[list[WritePlan], list[str]]:
    plans: list[WritePlan] = []
    skipped: list[str] = []

    targets = {
        device_dir / "arm_module.py": arm_module_py(robot_device),
        device_dir / "robot_factory.py": robot_factory_py(robot_device, module_version),
        device_dir / "robot_module.py": robot_module_py(robot_device),
    }

    adapters_init = device_dir / "adapters" / "__init__.py"
    if force or _needs_adapters_init(adapters_init):
        targets[adapters_init] = adapters_init_py(robot_device)

    for path, content in targets.items():
        exists = path.is_file()
        if exists and not force:
            if path.name == "__init__.py" and path.parent.name == "adapters":
                if not _needs_adapters_init(path):
                    skipped.append(str(path.relative_to(device_dir)))
                    continue
            else:
                skipped.append(str(path.relative_to(device_dir)))
                continue
        plans.append(
            WritePlan(
                path=path,
                content=content if content.endswith("\n") else content + "\n",
                reason="created" if not path.exists() else "overwritten",
            )
        )
    return plans, skipped


def _needs_adapters_init(path: Path) -> bool:
    if not path.is_file():
        return True
    text = path.read_text(encoding="utf-8")
    return "MoveItBackend" not in text


def infer_module_version(device_dir: Path, override: str | None) -> str:
    if override:
        return override
    for assembly_path in runtime_assembly_files(device_dir):
        _, version = read_manifest_distribution_version(
            assembly_path.read_text(encoding="utf-8")
        )
        if version:
            return version
    return "0.1.0"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="生成领域机械臂 Module API 文件")
    parser.add_argument("--domain", required=True, type=Path)
    parser.add_argument("--device", required=True)
    parser.add_argument("--module-version", default="")
    parser.add_argument("--apply", action="store_true", help="写入文件；默认仅预览")
    parser.add_argument("--force", action="store_true", help="覆盖已存在文件")
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

    module_version = infer_module_version(
        record.device_dir, args.module_version.strip() or None
    )
    plans, skipped = build_write_plan(
        device_dir=record.device_dir,
        domain_pkg=record.domain_pkg,
        robot_device=record.device_id,
        module_version=module_version,
        force=args.force,
    )

    result = {
        "ok": True,
        "dry_run": not args.apply,
        "device_id": record.device_id,
        "domain_pkg": record.domain_pkg,
        "robot_module_path": record.robot_module_path,
        "module_version": module_version,
        "writes": [
            {
                "path": str(plan.path.relative_to(record.domain)).replace("\\", "/"),
                "reason": plan.reason,
            }
            for plan in plans
        ],
        "skipped_existing": skipped,
    }

    if args.apply:
        for plan in plans:
            plan.path.parent.mkdir(parents=True, exist_ok=True)
            plan.path.write_text(plan.content, encoding="utf-8", newline="\n")

    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
