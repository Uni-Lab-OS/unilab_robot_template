#!/usr/bin/env python3
"""一键把 unilab_robot_template 机械臂引用进领域仓。"""

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
from _card_scaffold import (  # noqa: E402
    _class_prefix,
    assess_arm_card,
    moveit_arm_card_path,
    scaffold_domain_card,
    scaffold_moveit_arm_card,
)
from _commissioning_scaffold import (  # noqa: E402
    assess_moveit_commissioning,
    patch_device_moveit_post_init,
    patch_rail_post_init,
    scaffold_moveit_commissioning,
)
from _domain_layout import detect_domain_package, locate_device  # noqa: E402
from _domain_owned_assets import scaffold_domain_owned_assets  # noqa: E402
from _domain_scaffold import infer_domain_pkg, scaffold_domain_package  # noqa: E402
from _graph_scaffold import scaffold_graph as write_graph_scaffold  # noqa: E402
from _patch import (  # noqa: E402
    ensure_pyproject_dependencies,
    patch_device_card_inheritance,
    patch_model_block,
    pip_install_editable,
    scaffold_device_py,
)
from _preview_scaffold import scaffold_preview_device  # noqa: E402
from _rail_scaffold import default_rail_stl_path, scaffold_rail  # noqa: E402


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
            f"领域自有模式需要 {model_yaml}；请先用 --vendor-urdf 拷贝 vendor 资产，"
            f"或手动放到 devices/{device_id}/models/"
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


def _resolve_domain(args: argparse.Namespace) -> tuple[Path, bool]:
    if args.new_domain is not None:
        return args.new_domain.resolve(), True
    if args.domain is not None:
        domain = args.domain.resolve()
        if not domain.is_dir():
            raise FileNotFoundError(f"领域仓不存在: {domain}（新建请用 --new-domain）")
        return domain, False
    raise ValueError("必须指定 --domain 或 --new-domain")


def _resolve_domain_pkg(domain: Path, *, greenfield: bool, override: str | None) -> str:
    if override:
        return override
    if greenfield:
        return infer_domain_pkg(domain)
    return detect_domain_package(domain)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="一键引用 unilab_robot_template 机械臂到领域仓（写 provider + digest + 依赖）"
    )
    domain_group = parser.add_mutually_exclusive_group(required=True)
    domain_group.add_argument("--domain", type=Path, help="已存在的领域仓根目录")
    domain_group.add_argument(
        "--new-domain",
        type=Path,
        help="在空目录 scaffold 最小导轨+机械臂领域包",
    )
    parser.add_argument(
        "--device",
        "--arm-device",
        dest="device",
        required=True,
        help="graph 里机械臂 @device id",
    )
    mode = parser.add_mutually_exclusive_group(required=False)
    mode.add_argument(
        "--catalog",
        metavar="SLUG",
        nargs="?",
        const="cr5",
        default=None,
        help="引用 template MoveIt catalog 型号（默认 cr5；greenfield 可省略）",
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
    parser.add_argument(
        "--rail-device",
        default="rail",
        help="graph / @device 导轨 id（默认 rail）",
    )
    parser.add_argument(
        "--rail-stl",
        type=Path,
        help="覆盖默认导轨视觉 mesh（默认 SZLab arm_slideway.stl）",
    )
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
        "--scaffold-graph",
        action="store_true",
        help="写/合并 graph（新建领域包时默认开启）",
    )
    parser.add_argument(
        "--skip-graph",
        action="store_true",
        help="不修改 graph（已有领域包只导入机械臂）",
    )
    parser.add_argument(
        "--graph-file",
        default="deployment/graphs/local-debug.json",
        help="相对领域仓根的 graph JSON 路径",
    )
    parser.add_argument(
        "--vendor-urdf",
        type=Path,
        help="domain-owned greenfield：只读拷贝 vendor URDF",
    )
    parser.add_argument(
        "--vendor-mesh-dir",
        type=Path,
        help="domain-owned greenfield：vendor mesh 目录",
    )
    parser.add_argument(
        "--with-card",
        action="store_true",
        help="（兼容旧参数）机械臂卡片现已默认自动检测并补齐",
    )
    parser.add_argument(
        "--skip-card",
        action="store_true",
        help="跳过机械臂卡片检测与补齐",
    )
    parser.add_argument(
        "--with-preview-scaffold",
        action="store_true",
        help="preview-catalog：从 docs/demo 模板生成 model/mounts/device/card 骨架",
    )
    parser.add_argument("--card-title", help="设备卡片标题")
    parser.add_argument("--device-title", help="Preview @device displayname / 卡片标题默认")
    args = parser.parse_args(argv)

    demo_index = "docs/demo/README.md"

    try:
        domain, greenfield = _resolve_domain(args)
    except (ValueError, FileNotFoundError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False))
        return 2

    if not args.catalog and not args.preview_catalog and not args.domain_owned:
        if greenfield:
            args.catalog = "cr5"
        else:
            print(
                json.dumps(
                    {
                        "ok": False,
                        "error": "已有领域仓须显式指定 --catalog、--preview-catalog 或 --domain-owned",
                    },
                    ensure_ascii=False,
                )
            )
            return 2

    rail_stl = (args.rail_stl or default_rail_stl_path(SCRIPT_DIR)).resolve()

    template = template_root(SCRIPT_DIR)
    if template is None and (args.catalog or args.preview_catalog):
        print(
            json.dumps(
                {"ok": False, "error": "找不到 unilab_robot_template 根目录"},
                ensure_ascii=False,
            )
        )
        return 2

    should_scaffold_graph = not args.skip_graph and (args.scaffold_graph or greenfield)
    should_ensure_card = not args.skip_card
    should_scaffold_moveit_arm_card = should_ensure_card and not args.preview_catalog
    report: dict[str, Any] = {
        "ok": True,
        "dry_run": not args.apply,
        "greenfield": greenfield,
        "demo_index": demo_index,
        "domain": str(domain),
        "device_id": args.device,
        "rail_device_id": args.rail_device,
        "default_models": {
            "arm_catalog": args.catalog or None,
            "rail_mesh": str(rail_stl),
        },
    }

    try:
        domain_pkg = _resolve_domain_pkg(
            domain,
            greenfield=greenfield,
            override=args.domain_pkg,
        )
        report["domain_pkg"] = domain_pkg

        if greenfield:
            if args.apply:
                domain.mkdir(parents=True, exist_ok=True)
            report["domain_scaffold"] = scaffold_domain_package(
                domain,
                domain_pkg=domain_pkg,
                domain_owned=bool(args.domain_owned),
                graph_rel=args.graph_file,
                apply=args.apply,
            )
            report["rail_scaffold"] = scaffold_rail(
                domain,
                domain_pkg=domain_pkg,
                rail_id=args.rail_device,
                rail_stl=rail_stl,
                apply=args.apply,
                with_rail_simulation=should_scaffold_moveit_arm_card,
            )
            if should_scaffold_graph:
                report["graph_scaffold"] = write_graph_scaffold(
                    domain,
                    domain_pkg=domain_pkg,
                    rail_id=args.rail_device,
                    arm_id=args.device,
                    graph_rel=args.graph_file,
                    apply=args.apply,
                )
        elif should_scaffold_graph:
            report["rail_scaffold"] = scaffold_rail(
                domain,
                domain_pkg=domain_pkg,
                rail_id=args.rail_device,
                rail_stl=rail_stl,
                apply=args.apply,
                with_rail_simulation=should_scaffold_moveit_arm_card,
            )
            report["graph_scaffold"] = write_graph_scaffold(
                domain,
                domain_pkg=domain_pkg,
                rail_id=args.rail_device,
                arm_id=args.device,
                graph_rel=args.graph_file,
                apply=args.apply,
            )

        if args.domain_owned and args.vendor_urdf is not None:
            report["vendor_assets"] = scaffold_domain_owned_assets(
                domain,
                domain_pkg=domain_pkg,
                device_id=args.device,
                vendor_urdf=args.vendor_urdf,
                vendor_mesh_dir=args.vendor_mesh_dir,
                apply=args.apply,
            )

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

    mode_name = "domain-owned"
    if args.catalog:
        mode_name = "catalog-moveit"
    elif args.preview_catalog:
        mode_name = "catalog-preview"

    report.update(
        {
            "mode": mode_name,
            "device_py": str(target.device_py.relative_to(domain)).replace("\\", "/"),
            "model": model,
            "provider_one_liner": model["provider"],
        }
    )

    pyproject = domain / "pyproject.toml"
    deps_added: list[str] = []
    if args.catalog and entry is not None:
        report["template_root"] = str(template)
        report["catalog_distribution"] = entry.distribution
        if greenfield:
            report["demo_reference"] = "docs/demo/minimal-rail-arm-catalog"
        else:
            report["demo_reference"] = "docs/demo/catalog-moveit-cr5"
        deps = catalog_dependencies(entry)
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
        if greenfield and args.vendor_urdf is not None:
            report["demo_reference"] = "docs/demo/minimal-rail-arm-domain-owned"
        else:
            report["demo_reference"] = "docs/demo/domain-owned-moveit"

    card_assessment = assess_arm_card(
        domain,
        domain_pkg=domain_pkg,
        device_id=args.device,
        device_py=target.device_py,
        preview_mode=bool(args.preview_catalog),
    )

    if should_scaffold_moveit_arm_card:
        moveit_demo_dir = (
            template / "skills" / "introduce-unilab-robot" / "docs" / "demo" / "catalog-moveit-cr5"
            if template is not None
            else SCRIPT_DIR.parent / "docs" / "demo" / "catalog-moveit-cr5"
        )
        report["moveit_arm_card"] = scaffold_moveit_arm_card(
            domain,
            domain_pkg=domain_pkg,
            device_id=args.device,
            demo_dir=moveit_demo_dir,
            apply=args.apply,
        )
        if not args.skip_card:
            report["moveit_commissioning"] = scaffold_moveit_commissioning(
                domain,
                domain_pkg=domain_pkg,
                device_id=args.device,
                rail_id=args.rail_device,
                class_prefix=_class_prefix(args.device),
                demo_dir=moveit_demo_dir,
                apply=args.apply,
            )
            rail_py = domain / domain_pkg / "devices" / f"{args.rail_device}.py"
            if rail_py.is_file() and args.apply:
                rail_source = rail_py.read_text(encoding="utf-8")
                rail_updated, rail_changed = patch_rail_post_init(
                    rail_source,
                    domain_pkg=domain_pkg,
                )
                if rail_changed:
                    rail_py.write_text(rail_updated, encoding="utf-8", newline="\n")
                    report["rail_post_init_patched"] = True

        commissioning_assessment = assess_moveit_commissioning(
            domain,
            domain_pkg=domain_pkg,
            device_id=args.device,
            rail_id=args.rail_device,
            arm_card_path=moveit_arm_card_path(domain, domain_pkg, args.device),
        )
        report["commissioning_assessment"] = commissioning_assessment
        if not commissioning_assessment["complete"]:
            merged_missing = list(card_assessment.get("missing", []))
            merged_missing.extend(commissioning_assessment["missing"])
            card_assessment = {
                **card_assessment,
                "complete": False,
                "missing": merged_missing,
                "commissioning_complete": False,
            }
        else:
            card_assessment = {
                **card_assessment,
                "commissioning_complete": True,
            }

    report["card_assessment"] = card_assessment

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
        if should_scaffold_moveit_arm_card:
            card_prefix = _class_prefix(args.device)
            inherited, card_changed = patch_device_card_inheritance(
                updated,
                domain_pkg=domain_pkg,
                device_id=args.device,
                card_class_prefix=card_prefix,
            )
            updated = inherited
            changed = changed or card_changed
            if card_changed:
                report["device_card_inheritance_patched"] = True
        if should_scaffold_moveit_arm_card and not args.skip_card:
            moveit_updated, moveit_changed = patch_device_moveit_post_init(
                updated,
                domain_pkg=domain_pkg,
                device_id=args.device,
            )
            updated = moveit_updated
            changed = changed or moveit_changed
            if moveit_changed:
                report["device_post_init_patched"] = True
        report["device_changed"] = changed
        if args.apply and changed:
            target.device_py.write_text(updated, encoding="utf-8", newline="\n")
    else:
        assert target.class_name is not None
        card_prefix = _class_prefix(args.device) if should_scaffold_moveit_arm_card else None
        scaffold = scaffold_device_py(
            device_id=args.device,
            class_name=target.class_name,
            model=model,
            with_card=should_scaffold_moveit_arm_card,
            domain_pkg=domain_pkg,
            card_class_prefix=card_prefix,
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

    if should_ensure_card:
        card_title = args.card_title or f"{args.device} 机械臂调试卡片"
        card_report = scaffold_domain_card(
            domain,
            domain_pkg=domain_pkg,
            device_id=args.device,
            title=card_title,
            apply=args.apply,
        )
        report["card"] = card_report
        card_created = bool(card_report.get("created_files")) or bool(
            (report.get("moveit_arm_card") or {}).get("created_files")
        )
        card_patched = bool(report.get("device_card_inheritance_patched"))
        report["card_ensure"] = {
            "required": True,
            "was_complete": card_assessment["complete"],
            "missing_before": card_assessment["missing"],
            "created_or_patched": card_created or card_patched,
        }
        if args.apply and (card_created or card_patched):
            report["card_scaffolded"] = True
        elif not args.apply and not card_assessment["complete"]:
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
