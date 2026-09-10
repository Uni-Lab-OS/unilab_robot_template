"""Skill CLI 脚本单元测试。"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

SKILL_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = SKILL_ROOT / "scripts"
DOMAIN = SKILL_ROOT.parent / "Uni-Lab-SZLab"
DEVICE = "szlab_mixer_robot"

sys.path.insert(0, str(SCRIPTS))
from _domain_scan import (  # noqa: E402
    find_arm_device,
    patch_manifest_arm_module,
    read_manifest_arm_module,
    runtime_assembly_files,
)
from _model_facts import read_arm_model_facts  # noqa: E402


def _run(script: str, *args: str) -> tuple[int, dict]:
    proc = subprocess.run(
        [sys.executable, str(SCRIPTS / script), *args],
        capture_output=True,
        text=True,
        check=False,
    )
    payload = json.loads(proc.stdout)
    return proc.returncode, payload


def test_find_szlab_mixer_robot_precheck() -> None:
    if not DOMAIN.is_dir():
        return
    code, payload = _run(
        "check_domain_robot_module.py",
        "--domain",
        str(DOMAIN),
        "--device",
        DEVICE,
    )
    assert payload["device_id"] == DEVICE
    assert payload["domain_pkg"] == "szlab_poly_studio"
    assert payload["files"]["moveit_model.py"] is True
    assert payload["files"]["robot_module.py"] is True
    assert code == 0


def test_scaffold_dry_run_szlab() -> None:
    if not DOMAIN.is_dir():
        return
    code, payload = _run(
        "scaffold_domain_robot_module.py",
        "--domain",
        str(DOMAIN),
        "--device",
        DEVICE,
    )
    assert code == 0
    assert payload["dry_run"] is True
    assert payload["module_version"] == "0.1.0"
    assert not payload["writes"]
    assert any("robot_module.py" in item for item in payload["skipped_existing"])


def test_manifest_regex_szlab() -> None:
    if not DOMAIN.is_dir():
        return
    record = find_arm_device(DOMAIN, DEVICE)
    assert record is not None
    assembly = runtime_assembly_files(record.device_dir)[0]
    text = assembly.read_text(encoding="utf-8")
    arm_module = read_manifest_arm_module(text)
    assert arm_module is not None
    assert arm_module.endswith(".robot_module")
    updated, changed = patch_manifest_arm_module(
        text,
        provider_module=record.provider_module,
        robot_module_path=record.robot_module_path,
    )
    assert changed is False
    assert read_manifest_arm_module(updated) == record.robot_module_path


def test_fix_manifest_dry_run_szlab() -> None:
    if not DOMAIN.is_dir():
        return
    code, payload = _run(
        "fix_runtime_manifest.py",
        "--domain",
        str(DOMAIN),
        "--device",
        DEVICE,
    )
    assert payload["robot_module_path"].endswith(".robot_module")
    patch = payload["patches"][0]
    assert patch["before"].endswith(".robot_module")
    assert patch["after"].endswith(".robot_module")
    assert patch["changed"] is False
    assert code == 0


def test_fix_moveit_runtime_attach_dry_run_szlab() -> None:
    if not DOMAIN.is_dir():
        return
    record = find_arm_device(DOMAIN, DEVICE)
    assert record is not None
    facts = read_arm_model_facts(
        moveit_model_path=record.device_dir / "moveit_model.py",
        domain_pkg=record.domain_pkg,
        device_id=record.device_id,
    )
    code, payload = _run(
        "fix_moveit_runtime_attach.py",
        "--domain",
        str(DOMAIN),
        "--device",
        DEVICE,
    )
    assert payload["arm_endpoint"] == facts.arm_endpoint
    assert payload["robot_module_path"].endswith(".robot_module")
    assert payload["expected_end_effector"] == f"{DEVICE}_{facts.flange_frame}"
    assert payload["expected_model_ref"].endswith("/models/model.yaml")
    assert payload["model_ref_aligned"] is True
    assert isinstance(payload["patches"], list)
    assert payload["dry_run"] is True


def test_moveit_attach_contract_szlab() -> None:
    if not DOMAIN.is_dir():
        return
    code, payload = _run(
        "check_moveit_attach_contract.py",
        "--domain",
        str(DOMAIN),
        "--device",
        DEVICE,
    )
    assert payload["expected_model_ref"].endswith("/models/model.yaml")
    assert payload["current_model_ref"] == payload["expected_model_ref"]
    assert code == 0
    assert payload["ok"] is True


def test_moveit_joint_alignment_szlab() -> None:
    if not DOMAIN.is_dir():
        return
    record = find_arm_device(DOMAIN, DEVICE)
    assert record is not None
    moveit_model = record.device_dir / "moveit_model.py"
    if not moveit_model.is_file():
        return
    code, payload = _run(
        "check_moveit_joint_alignment.py",
        "--domain",
        str(DOMAIN),
        "--device",
        DEVICE,
    )
    assert payload["arm_endpoint"].endswith(":arm")
    assert len(payload["canonical_joint_names"]) == 6
    expected_qualified = payload["expected_qualified_joint_names"]
    assert expected_qualified[0] == f"{DEVICE}_joint_1"
    assert "remediation" in payload
    assert payload["remediation"]["qualified_joint_names"][0] == f"{DEVICE}_joint_1"
    legacy = payload.get("domain_legacy_qualified_literals") or []
    if payload["canonical_joint_names"][0].startswith("cr7_") or legacy:
        assert code == 1
        assert payload["errors"]
    else:
        assert code == 0
        assert list(payload["canonical_joint_names"]) == [
            f"joint_{index}" for index in range(1, 7)
        ]


def test_scaffold_adapters_dry_run_szlab() -> None:
    if not DOMAIN.is_dir():
        return
    code, payload = _run(
        "scaffold_domain_adapters.py",
        "--domain",
        str(DOMAIN),
        "--device",
        DEVICE,
    )
    assert code == 0
    assert payload["dry_run"] is True
