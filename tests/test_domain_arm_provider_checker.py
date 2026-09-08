from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def _load_checker():
    path = (
        Path(__file__).resolve().parents[1]
        / ".cursor/skills/use-unilab-arm-package/scripts/check_domain_arm_assembly.py"
    )
    module_name = "check_domain_arm_assembly_test"
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def test_checker_accepts_domain_arm_provider() -> None:
    checker = _load_checker()
    report = checker.Report(domain="test")
    checker._check_arm_model(
        checker.DeviceModel(
            path="devices/robot.py",
            device_id="my_robot",
            class_name="MyRobot",
            model={
                "type": "package_moveit",
                "provider": "my_lab_arm:build_moveit_model",
                "source_digest": "a" * 64,
            },
        ),
        report,
    )
    assert report.ok
    assert report.warnings


def test_checker_still_accepts_catalog_arm_provider() -> None:
    checker = _load_checker()
    report = checker.Report(domain="test")
    checker._check_arm_model(
        checker.DeviceModel(
            path="devices/robot.py",
            device_id="my_robot",
            class_name="MyRobot",
            model={
                "type": "package_moveit",
                "provider": "unilab_arm_cr7:build_moveit_model",
                "source_digest": "b" * 64,
            },
        ),
        report,
    )
    assert report.ok
    assert not report.warnings
