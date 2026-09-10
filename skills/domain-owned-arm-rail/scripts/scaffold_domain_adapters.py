#!/usr/bin/env python3
"""生成领域 device 的 MoveIt adapters（不写死 catalog 型号关节名）。"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from _domain_scan import find_arm_device  # noqa: E402
from _templates import adapters_init_py, adapters_moveit_py  # noqa: E402

_COPY_FROM_TEMPLATE = (
    "_support.py",
    "moveit_client.py",
    "moveit_commissioning.py",
)
_GENERATED = {
    "__init__.py": adapters_init_py,
    "moveit.py": adapters_moveit_py,
}


def _template_adapter_dir() -> Path:
    repo_root = SCRIPT_DIR.parent.parent
    candidates = (
        repo_root
        / "unilab_robot_template"
        / "packages"
        / "unilab-arm-cr7"
        / "src"
        / "unilab_arm_cr7"
        / "adapters",
        repo_root
        / "unilab_robot_template"
        / "packages"
        / "unilab-arm-cr5"
        / "src"
        / "unilab_arm_cr5"
        / "adapters",
    )
    for path in candidates:
        if path.is_dir():
            return path
    raise FileNotFoundError("未找到 template MoveIt adapters（unilab-arm-cr7）")


def build_adapter_copy_plan(
    *,
    device_dir: Path,
    device_id: str,
    force: bool,
) -> tuple[list[dict[str, str]], list[str]]:
    source_dir = _template_adapter_dir()
    target_dir = device_dir / "adapters"
    writes: list[dict[str, str]] = []
    skipped: list[str] = []

    for name in _COPY_FROM_TEMPLATE:
        source = source_dir / name
        target = target_dir / name
        if not source.is_file():
            raise FileNotFoundError(f"template adapter 缺失: {source}")
        if target.is_file() and not force:
            skipped.append(str(target.relative_to(device_dir)))
            continue
        writes.append(
            {
                "path": str(target.relative_to(device_dir)).replace("\\", "/"),
                "source": str(source),
                "kind": "copy",
            }
        )

    for name, builder in _GENERATED.items():
        target = target_dir / name
        if target.is_file() and not force:
            skipped.append(str(target.relative_to(device_dir)))
            continue
        writes.append(
            {
                "path": str(target.relative_to(device_dir)).replace("\\", "/"),
                "source": "generated-from-model-descriptor",
                "kind": "generate",
            }
        )
    return writes, skipped


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="生成领域 MoveIt adapters")
    parser.add_argument("--domain", required=True, type=Path)
    parser.add_argument("--device", required=True)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--force", action="store_true")
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

    writes, skipped = build_adapter_copy_plan(
        device_dir=record.device_dir,
        device_id=record.device_id,
        force=args.force,
    )
    result = {
        "ok": True,
        "dry_run": not args.apply,
        "template_adapter_dir": str(_template_adapter_dir()),
        "writes": writes,
        "skipped_existing": skipped,
    }
    if args.apply:
        target_dir = record.device_dir / "adapters"
        target_dir.mkdir(parents=True, exist_ok=True)
        source_dir = _template_adapter_dir()
        for item in writes:
            target = record.device_dir / item["path"]
            if item["kind"] == "copy":
                shutil.copy2(source_dir / target.name, target)
            else:
                builder = _GENERATED[target.name]
                content = builder(record.device_id)
                target.write_text(
                    content if content.endswith("\n") else content + "\n",
                    encoding="utf-8",
                    newline="\n",
                )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
