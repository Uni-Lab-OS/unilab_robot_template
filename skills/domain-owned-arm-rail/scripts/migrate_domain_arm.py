#!/usr/bin/env python3
"""领域机械臂 Module API 一键移植（唯一入口）。"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent


def _run_path(script_path: Path, args: list[str]) -> tuple[int, dict | str]:
    command = [sys.executable, str(script_path), *args]
    proc = subprocess.run(command, capture_output=True, text=True, check=False)
    stdout = proc.stdout.strip()
    if not stdout:
        return proc.returncode, proc.stderr.strip() or {}
    try:
        payload: dict | str = json.loads(stdout)
    except json.JSONDecodeError:
        payload = stdout
    return proc.returncode, payload


def _run(script: str, args: list[str]) -> tuple[int, dict | str]:
    return _run_path(SCRIPT_DIR / script, args)


def _assembly_checker_path() -> Path | None:
    checker = Path("use-unilab-arm-package") / "scripts" / "check_domain_arm_assembly.py"
    relatives = (
        Path("unilab_robot_template") / "skills" / checker,
        Path("skills") / checker,
    )
    for parent in SCRIPT_DIR.parents:
        for relative in relatives:
            candidate = parent / relative
            if candidate.is_file():
                return candidate
    sibling = SCRIPT_DIR.parent.parent.parent / checker
    return sibling if sibling.is_file() else None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "领域机械臂一键移植：robot_module + manifest + attach 链；"
            "关节名门禁要求 canonical=joint_N、qualified={device_id}_joint_N"
        )
    )
    parser.add_argument("--domain", required=True, type=Path, help="领域仓根目录")
    parser.add_argument("--device", required=True, help="@device 机械臂 id")
    parser.add_argument("--module-version", default="", help="默认从 manifest 读取")
    parser.add_argument(
        "--apply",
        action="store_true",
        help="写入领域仓；不加则仅预览 JSON",
    )
    parser.add_argument("--force", action="store_true", help="覆盖已有 scaffold 文件")
    args = parser.parse_args(argv)

    domain = str(args.domain.resolve())
    common = ["--domain", domain, "--device", args.device]
    if args.module_version.strip():
        common.extend(["--module-version", args.module_version.strip()])

    steps: list[dict[str, object]] = []

    if not args.apply:
        sc_code, sc = _run("scaffold_domain_robot_module.py", common)
        steps.append({"step": "preview_scaffold", "exit_code": sc_code, "result": sc})
        fx_code, fx = _run("fix_runtime_manifest.py", common)
        steps.append({"step": "preview_manifest", "exit_code": fx_code, "result": fx})
        ad_code, ad = _run("scaffold_domain_adapters.py", common)
        steps.append({"step": "preview_adapters", "exit_code": ad_code, "result": ad})
        attach_fix_code, attach_fix = _run("fix_moveit_runtime_attach.py", common)
        steps.append(
            {
                "step": "preview_moveit_runtime_attach",
                "exit_code": attach_fix_code,
                "result": attach_fix,
            }
        )
        joint_fix_code, joint_fix = _run("fix_device_joint_names.py", common)
        steps.append(
            {
                "step": "preview_fix_device_joint_names",
                "exit_code": joint_fix_code,
                "result": joint_fix,
            }
        )
        joint_code, joint = _run("check_moveit_joint_alignment.py", common)
        steps.append(
            {
                "step": "preview_moveit_joint_alignment",
                "exit_code": joint_code,
                "result": joint,
            }
        )
        attach_code, attach = _run("check_moveit_attach_contract.py", common)
        steps.append(
            {
                "step": "preview_moveit_attach_contract",
                "exit_code": attach_code,
                "result": attach,
            }
        )
        ok = (
            sc_code == 0
            and attach_fix_code == 0
            and joint_code == 0
            and attach_code == 0
        )
        print(
            json.dumps(
                {
                    "ok": ok,
                    "applied": False,
                    "device": args.device,
                    "domain": domain,
                    "message": "预览完成；加 --apply 执行一键移植",
                    "steps": steps,
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 0 if ok else 1

    joint_fix_code, joint_fix = _run("fix_device_joint_names.py", [*common, "--apply"])
    steps.append(
        {
            "step": "fix_device_joint_names",
            "exit_code": joint_fix_code,
            "result": joint_fix,
        }
    )

    scaffold_args = [*common, "--apply"]
    if args.force:
        scaffold_args.append("--force")
    sc_code, sc = _run("scaffold_domain_robot_module.py", scaffold_args)
    steps.append({"step": "scaffold", "exit_code": sc_code, "result": sc})

    fx_code, fx = _run("fix_runtime_manifest.py", [*common, "--apply"])
    steps.append({"step": "fix_manifest", "exit_code": fx_code, "result": fx})

    ad_code, ad = _run("scaffold_domain_adapters.py", [*common, "--apply"])
    steps.append({"step": "scaffold_adapters", "exit_code": ad_code, "result": ad})

    attach_fix_code, attach_fix = _run(
        "fix_moveit_runtime_attach.py",
        [*common, "--apply"],
    )
    steps.append(
        {
            "step": "fix_moveit_runtime_attach",
            "exit_code": attach_fix_code,
            "result": attach_fix,
        }
    )

    post_code, post = _run(
        "check_domain_robot_module.py",
        [*common, "--import-test"],
    )
    steps.append({"step": "verify", "exit_code": post_code, "result": post})

    joint_code, joint = _run(
        "check_moveit_joint_alignment.py",
        common,
    )
    steps.append(
        {
            "step": "verify_moveit_joint_alignment",
            "exit_code": joint_code,
            "result": joint,
        }
    )

    attach_code, attach = _run(
        "check_moveit_attach_contract.py",
        common,
    )
    steps.append(
        {
            "step": "verify_moveit_attach_contract",
            "exit_code": attach_code,
            "result": attach,
        }
    )

    assembly_checker = _assembly_checker_path()
    if assembly_checker is None:
        assembly_code, assembly = 2, {
            "ok": False,
            "error": "缺少模板仓 check_domain_arm_assembly.py，无法完成整机装配门禁",
        }
    else:
        assembly_code, assembly = _run_path(assembly_checker, ["--domain", domain])
    steps.append(
        {
            "step": "verify_domain_arm_assembly",
            "exit_code": assembly_code,
            "result": assembly,
        }
    )

    ok = (
        joint_fix_code == 0
        and sc_code == 0
        and fx_code == 0
        and ad_code == 0
        and attach_fix_code == 0
        and post_code == 0
        and joint_code == 0
        and attach_code == 0
        and assembly_code == 0
    )
    print(
        json.dumps(
            {
                "ok": ok,
                "applied": True,
                "device": args.device,
                "domain": domain,
                "message": "移植完成" if ok else "移植失败，见 steps.verify.result.errors",
                "steps": steps,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
