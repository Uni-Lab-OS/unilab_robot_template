#!/usr/bin/env python3
"""检查领域机械臂 Module API 与 runtime manifest 是否就绪。"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from _domain_scan import (  # noqa: E402
    find_arm_device,
    manifest_points_to_moveit_model,
    read_manifest_arm_module,
    read_manifest_distribution_version,
    runtime_assembly_files,
)
from _module_loader import load_robot_module  # noqa: E402


@dataclass
class Report:
    ok: bool = True
    device_id: str = ""
    domain_pkg: str = ""
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    files: dict[str, bool] = field(default_factory=dict)
    manifest: dict[str, str | None] = field(default_factory=dict)


def check_record(
    record,
    *,
    import_test: bool,
    expected_version: str | None,
) -> Report:
    report = Report(device_id=record.device_id, domain_pkg=record.domain_pkg)
    device_dir = record.device_dir

    required = {
        "moveit_model.py": device_dir / "moveit_model.py",
        "robot_module.py": device_dir / "robot_module.py",
        "robot_factory.py": device_dir / "robot_factory.py",
        "arm_module.py": device_dir / "arm_module.py",
    }
    for name, path in required.items():
        exists = path.is_file()
        report.files[name] = exists
        if not exists:
            report.errors.append(f"缺少文件: {path.relative_to(record.domain)}")

    adapters_init = device_dir / "adapters" / "__init__.py"
    if adapters_init.is_file():
        text = adapters_init.read_text(encoding="utf-8")
        if "MoveItBackend" not in text:
            report.errors.append("adapters/__init__.py 未导出 MoveItBackend")
    else:
        report.warnings.append("缺少 adapters/__init__.py")

    assemblies = runtime_assembly_files(device_dir)
    if not assemblies:
        report.warnings.append("未找到 moveit_runtime_assembly / *runtime_assembly*.py")
    for assembly_path in assemblies:
        text = assembly_path.read_text(encoding="utf-8")
        arm_module = read_manifest_arm_module(text)
        rel = str(assembly_path.relative_to(record.domain)).replace("\\", "/")
        report.manifest[rel] = arm_module
        if manifest_points_to_moveit_model(text, record.provider_module):
            report.errors.append(
                f"{rel}: arm manifest 仍指向 moveit_model，应为 {record.robot_module_path}"
            )
        elif arm_module != record.robot_module_path:
            report.errors.append(
                f"{rel}: arm manifest={arm_module!r}，期望 {record.robot_module_path!r}"
            )

    if import_test and (device_dir / "robot_module.py").is_file():
        try:
            module = load_robot_module(
                domain=record.domain,
                domain_pkg=record.domain_pkg,
                device_dir=device_dir,
                module_name=record.robot_module_path,
            )
        except Exception as exc:  # noqa: BLE001
            report.errors.append(f"import {record.robot_module_path} 失败: {exc}")
        else:
            if getattr(module, "MODULE_API_VERSION", None) != 1:
                report.errors.append("robot_module.MODULE_API_VERSION != 1")
            if str(getattr(module, "MODULE_KIND", "")) != "arm":
                report.errors.append("robot_module.MODULE_KIND != arm")
            version = str(getattr(module, "__version__", ""))
            if expected_version and version != expected_version:
                report.errors.append(
                    f"robot_module.__version__={version!r}，期望 {expected_version!r}"
                )
            elif not expected_version:
                manifest_version = _manifest_module_version(device_dir)
                if manifest_version and version != manifest_version:
                    report.errors.append(
                        f"robot_module.__version__={version!r}，manifest {manifest_version!r}"
                    )
            for symbol in (
                "create_arm_module",
                "create_moveit_backend",
                "build_moveit_model",
                "MODEL_DESCRIPTOR",
            ):
                if symbol == "MODEL_DESCRIPTOR":
                    if not hasattr(module, symbol):
                        report.errors.append("robot_module 缺少 MODEL_DESCRIPTOR")
                elif not callable(getattr(module, symbol, None)):
                    report.errors.append(f"robot_module 缺少可调用 {symbol}")

    report.ok = not report.errors
    return report


def _manifest_module_version(device_dir: Path) -> str | None:
    for assembly_path in runtime_assembly_files(device_dir):
        _, version = read_manifest_distribution_version(
            assembly_path.read_text(encoding="utf-8")
        )
        if version:
            return version
    return None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="检查领域机械臂 Module API 就绪状态")
    parser.add_argument("--domain", required=True, type=Path)
    parser.add_argument("--device", required=True)
    parser.add_argument("--import-test", action="store_true")
    parser.add_argument("--module-version", default="")
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

    report = check_record(
        record,
        import_test=args.import_test,
        expected_version=args.module_version.strip() or None,
    )
    print(
        json.dumps(
            {
                "ok": report.ok,
                "device_id": report.device_id,
                "domain_pkg": report.domain_pkg,
                "files": report.files,
                "manifest": report.manifest,
                "errors": report.errors,
                "warnings": report.warnings,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
