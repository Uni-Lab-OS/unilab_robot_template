#!/usr/bin/env python3
"""一键把 unilab_robot_template 机械臂型号引用进领域仓。"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from _catalog import (  # noqa: E402
    catalog_dependencies,
    load_catalog_entry,
    load_preview_catalog_entry,
    preview_catalog_dependencies,
    read_preview_source_digest,
    read_source_digest,
    template_root,
)
from _card_scaffold import scaffold_domain_card  # noqa: E402
from _preview_scaffold import scaffold_preview_device  # noqa: E402
from _domain_layout import detect_domain_package, locate_device  # noqa: E402
from _patch import (  # noqa: E402
    ensure_pyproject_dependencies,
    patch_model_block,
    pip_install_editable,
    scaffold_device_py,
)


def _build_catalog_model(entry_slug: str, template: Path) -> dict[str, Any]:
    entry = load_catalog_entry(template, entry_slug)
    return {
        "type": "package_moveit",
        "provider": entry.provider,
        "source_digest": read_source_digest(entry.model_yaml),
    }


def _build_preview_model(domain_pkg: str, device_id: str, digest: str) -> dict[str, Any]:
    provider_base = f"{domain_pkg}.devices.{device_id}.model:build_base"
    provider_kin = f"{domain_pkg}.devices.{device_id}.model:build_kinematics"
    return {
        "type": "package_static",
        "provider": provider_base,
        "source_digest": digest,
        "joint_state_provider": provider_kin,
        "joint_state_source_digest": digest,
    }


def _build_domain_model(domain: Path, domain_pkg: str, device_id: str) -> dict[str, Any]:
    model_yaml = domain / domain_pkg / "devices" / device_id / "models" / "model.yaml"
    if not model_yaml.is_file():
        raise FileNotFoundError(
            f"领域自有模式需要 {model_yaml}；请先把 vendor URDF/mesh 放到 devices/{device_id}/models/"
        )
    digest = read_source_digest(model_yaml)
    provider = f"{domain_pkg}.devices.{device_id}.moveit_model:build_moveit_model"
    return {
        "type": "package_moveit",
        "model_ownership": "domain",
        "provider": provider,
        "source_digest": digest,
    }


def _run_checker(domain: Path) -> tuple[int, dict[str, Any] | str]:
    checker = (
        template_root(SCRIPT_DIR)
        or SCRIPT_DIR.parents[2]
    ) / "skills" / "use-unilab-arm-package" / "scripts" / "check_domain_arm_assembly.py"
    if not checker.is_file():
        return 0, {"skipped": True, "reason": "checker not found"}
    proc = subprocess.run(
        [sys.executable, str(checker), "--domain", str(domain)],
        capture_output=True,
        text=True,
        check=False,
    )
    stdout = proc.stdout.strip()
    try:
        payload: dict[str, Any] | str = json.loads(stdout) if stdout else {}
    except json.JSONDecodeError:
        payload = stdout or proc.stderr.strip()
    return proc.returncode, payload


def _run_migrate(domain: Path, device_id: str) -> tuple[int, dict[str, Any] | str]:
    migrate = (
        template_root(SCRIPT_DIR)
        or SCRIPT_DIR.parents[2]
    ) / "skills" / "domain-owned-arm-rail" / "scripts" / "migrate_domain_arm.py"
    if not migrate.is_file():
        migrate = (
            template_root(SCRIPT_DIR.parent.parent.parent)
            or Path("domain-owned-arm-rail-skill")
        ) / "scripts" / "migrate_domain_arm.py"
    proc = subprocess.run(
        [
            sys.executable,
            str(migrate),
            "--domain",
            str(domain),
            "--device",
            device_id,
            "--apply",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    stdout = proc.stdout.strip()
    try:
        payload: dict[str, Any] | str = json.loads(stdout) if stdout else {}
    except json.JSONDecodeError:
        payload = stdout or proc.stderr.strip()
    return proc.returncode, payload


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="一键引用 unilab_robot_template 机械臂到领域仓（写 provider + digest + 依赖）"
    )
    parser.add_argument("--domain", required=True, type=Path, help="领域仓根目录")
    parser.add_argument("--device", required=True, help="图里 @device id")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        "--catalog",
        metavar="SLUG",
        help="引用 template MoveIt catalog 型号（cr5 / cr7）",
    )
    mode.add_argument(
        "--preview-catalog",
        metavar="SLUG",
        help="引用 template Preview catalog 型号（elite-cs66）",
    )
    mode.add_argument(
        "--domain-owned",
        action="store_true",
        help="引用领域仓 devices/<device>/moveit_model.py",
    )
    parser.add_argument("--domain-pkg", help="覆盖自动探测的 Python 包名")
    parser.add_argument("--apply", action="store_true", help="写入 pyproject 与 device.py")
    parser.add_argument(
        "--install",
        action="store_true",
        help="catalog 模式下 pip install -e template 对应 L1 包",
    )
    parser.add_argument(
        "--migrate",
        action="store_true",
        help="domain-owned 模式下继续跑 migrate_domain_arm.py --apply",
    )
    parser.add_argument("--skip-check", action="store_true", help="跳过装配检查器")
    parser.add_argument(
        "--with-card",
        action="store_true",
        help="生成 frontend/cards/<device>-card manifest（引用 template 卡片）",
    )
    parser.add_argument(
        "--with-preview-scaffold",
        action="store_true",
        help="preview-catalog：从 docs/demo 模板生成 model/mounts/device/card 骨架",
    )
    parser.add_argument("--card-title", help="设备卡片标题")
    parser.add_argument("--device-title", help="Preview @device displayname / 卡片标题默认")
    args = parser.parse_args(argv)

    domain = args.domain.resolve()
    if not domain.is_dir():
        print(json.dumps({"ok": False, "error": f"领域仓不存在: {domain}"}, ensure_ascii=False))
        return 2

    template = template_root(SCRIPT_DIR)
    if template is None and (args.catalog or args.preview_catalog):
        print(
            json.dumps(
                {"ok": False, "error": "找不到 unilab_robot_template 根目录"},
                ensure_ascii=False,
            )
        )
        return 2

    demo_index = "docs/demo/README.md"

    try:
        domain_pkg = args.domain_pkg or detect_domain_package(domain)
        target = locate_device(domain, args.device, domain_pkg)
        preview_entry = None
        entry = None
        if args.catalog:
            assert template is not None
            entry = load_catalog_entry(template, args.catalog)
            model = _build_catalog_model(args.catalog, template)
        elif args.preview_catalog:
            assert template is not None
            preview_entry = load_preview_catalog_entry(template, args.preview_catalog)
            entry = None
            digest = read_preview_source_digest(template, args.preview_catalog)
            model = _build_preview_model(domain_pkg, args.device, digest)
        else:
            model = _build_domain_model(domain, domain_pkg, args.device)
    except (ValueError, FileNotFoundError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False))
        return 2

    mode = "domain-owned"
    if args.catalog:
        mode = "catalog-moveit"
    elif args.preview_catalog:
        mode = "catalog-preview"

    report: dict[str, Any] = {
        "ok": True,
        "dry_run": not args.apply,
        "mode": mode,
        "demo_index": demo_index,
        "domain": str(domain),
        "domain_pkg": domain_pkg,
        "device_id": args.device,
        "device_py": str(target.device_py.relative_to(domain)).replace("\\", "/"),
        "model": model,
        "provider_one_liner": model["provider"],
    }

    pyproject = domain / "pyproject.toml"
    deps_added: list[str] = []
    if args.catalog and entry is not None:
        report["template_root"] = str(template)
        report["catalog_distribution"] = entry.distribution
        deps = catalog_dependencies(entry)
        report["demo_reference"] = "docs/demo/catalog-moveit-cr5"
        if pyproject.is_file():
            updated, deps_added = ensure_pyproject_dependencies(pyproject, deps)
            report["dependencies_added"] = deps_added
            if args.apply and deps_added:
                pyproject.write_text(updated, encoding="utf-8", newline="\n")
        else:
            report["dependencies_added"] = []
            report["warning"] = "未找到 pyproject.toml，请手动添加 catalog 依赖"
    elif args.preview_catalog and preview_entry is not None:
        assert template is not None
        report["template_root"] = str(template)
        report["catalog_distribution"] = preview_entry.distribution
        report["demo_reference"] = f"docs/demo/{preview_entry.demo_dir.name}"
        deps = preview_catalog_dependencies(preview_entry)
        if pyproject.is_file():
            updated, deps_added = ensure_pyproject_dependencies(pyproject, deps)
            report["dependencies_added"] = deps_added
            if args.apply and deps_added:
                pyproject.write_text(updated, encoding="utf-8", newline="\n")
        else:
            report["dependencies_added"] = []
            report["warning"] = "未找到 pyproject.toml，请手动添加 preview catalog 依赖"
    elif args.domain_owned:
        report["demo_reference"] = "docs/demo/domain-owned-moveit"

    device_exists = target.device_py.is_file()
    report["device_exists"] = device_exists
    use_preview_scaffold = bool(args.preview_catalog and args.with_preview_scaffold and preview_entry)
    if use_preview_scaffold:
        device_title = args.device_title or args.card_title or args.device.replace("_", " ").title()
        preview_report = scaffold_preview_device(
            domain,
            domain_pkg=domain_pkg,
            device_id=args.device,
            device_title=device_title,
            demo_dir=preview_entry.demo_dir,
            apply=args.apply,
        )
        report["preview_scaffold"] = preview_report
        report["device_changed"] = bool(preview_report.get("changed"))
        if device_exists and args.apply:
            original = target.device_py.read_text(encoding="utf-8")
            updated, changed = patch_model_block(original, model)
            if changed:
                target.device_py.write_text(updated, encoding="utf-8", newline="\n")
                report["device_model_patched"] = True
    elif device_exists:
        original = target.device_py.read_text(encoding="utf-8")
        updated, changed = patch_model_block(original, model)
        report["device_changed"] = changed
        if args.apply and changed:
            target.device_py.write_text(updated, encoding="utf-8", newline="\n")
    else:
        assert target.class_name is not None
        scaffold = scaffold_device_py(
            device_id=args.device,
            class_name=target.class_name,
            model=model,
        )
        report["device_changed"] = True
        if args.apply:
            target.device_py.parent.mkdir(parents=True, exist_ok=True)
            target.device_py.write_text(scaffold, encoding="utf-8", newline="\n")

    install_dir = None
    if args.catalog and entry is not None:
        install_dir = entry.package_dir
    elif args.preview_catalog and preview_entry is not None:
        install_dir = preview_entry.package_dir
    if args.apply and args.install and install_dir is not None:
        code, output = pip_install_editable(install_dir)
        report["pip_install"] = {"exit_code": code, "output": output[-2000:]}
        if code != 0:
            report["ok"] = False

    if args.apply and args.domain_owned and args.migrate:
        code, payload = _run_migrate(domain, args.device)
        report["migrate"] = {"exit_code": code, "payload": payload}
        if code != 0:
            report["ok"] = False

    if args.with_card:
        card_title = args.card_title or f"{args.device} 机械臂调试卡片"
        card_report = scaffold_domain_card(
            domain,
            domain_pkg=domain_pkg,
            device_id=args.device,
            title=card_title,
            apply=args.apply,
        )
        report["card"] = card_report
        if args.apply and card_report.get("created_files"):
            report["card_scaffolded"] = True

    if args.apply and not args.skip_check and not args.preview_catalog:
        code, payload = _run_checker(domain)
        report["checker"] = {"exit_code": code, "payload": payload}
        if code != 0:
            report["ok"] = False

    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
