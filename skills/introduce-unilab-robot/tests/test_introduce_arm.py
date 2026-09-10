from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "introduce_arm.py"
)


def _run(args: list[str]) -> dict:
    proc = subprocess.run(
        [sys.executable, str(SCRIPT), *args],
        capture_output=True,
        text=True,
        check=False,
    )
    payload = json.loads(proc.stdout)
    payload["_exit"] = proc.returncode
    return payload


def test_catalog_dry_run(tmp_path: Path) -> None:
    domain = tmp_path / "demo-lab"
    pkg = domain / "demo_lab" / "devices" / "my_robot"
    pkg.mkdir(parents=True)
    (domain / "pyproject.toml").write_text(
        """
[project]
dependencies = [
  "PyYAML>=6.0",
]
[tool.setuptools.packages.find]
include = ["demo_lab*"]
""".strip()
        + "\n",
        encoding="utf-8",
    )
    (pkg / "device.py").write_text(
        '''
from unilabos.registry.decorators import device

@device(
    id="my_robot",
    model={"type": "package_moveit", "provider": "unilab_arm_cr5:build_moveit_model", "source_digest": "0"*64},
)
class MyRobotDevice:
    pass
'''.strip()
        + "\n",
        encoding="utf-8",
    )
    report = _run(
        [
            "--domain",
            str(domain),
            "--device",
            "my_robot",
            "--catalog",
            "cr5",
        ]
    )
    assert report["_exit"] == 0
    assert report["provider_one_liner"] == "unilab_arm_cr5:build_moveit_model"
    assert report["dry_run"] is True
    assert len(report["model"]["source_digest"]) == 64


def test_scaffold_new_device(tmp_path: Path) -> None:
    domain = tmp_path / "demo-lab"
    domain.mkdir()
    (domain / "pyproject.toml").write_text(
        """
[project]
dependencies = []
[tool.setuptools.packages.find]
include = ["demo_lab*"]
""".strip()
        + "\n",
        encoding="utf-8",
    )
    (domain / "demo_lab" / "devices").mkdir(parents=True)
    report = _run(
        [
            "--domain",
            str(domain),
            "--device",
            "line_robot",
            "--catalog",
            "cr7",
            "--apply",
            "--skip-check",
        ]
    )
    assert report["_exit"] == 0
    device_py = domain / "demo_lab" / "devices" / "line_robot" / "device.py"
    assert device_py.is_file()
    text = device_py.read_text(encoding="utf-8")
    assert "unilab_arm_cr7:build_moveit_model" in text
    assert "unilab-arm-cr7" in (domain / "pyproject.toml").read_text(encoding="utf-8")
