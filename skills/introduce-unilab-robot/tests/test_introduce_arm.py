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
    assert report["card_assessment"]["complete"] is False
    assert report["card"]["changed"] is True
    assert report["moveit_arm_card"]["changed"] is True


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
    assert "LineRobotArmCardMixin" in text
    assert (domain / "demo_lab" / "devices" / "line_robot" / "line_robot_arm_card.py").is_file()
    assert (
        domain / "frontend" / "cards" / "line-robot-card" / "card.manifest.json"
    ).is_file()
    assert "unilab-arm-cr7" in (domain / "pyproject.toml").read_text(encoding="utf-8")


def test_catalog_auto_ensures_card_dry_run(tmp_path: Path) -> None:
    domain = tmp_path / "demo-lab"
    pkg = domain / "demo_lab" / "devices" / "my_robot"
    pkg.mkdir(parents=True)
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
            "--card-title",
            "Demo Robot Card",
        ]
    )
    assert report["_exit"] == 0
    card = report["card"]
    assert card["changed"] is True
    assert "frontend/cards/my-robot-card/card.manifest.json" in card["planned_files"]
    assert card["created_files"] == []
    assert report["card_ensure"]["was_complete"] is False


def test_catalog_auto_patches_existing_device_inheritance(tmp_path: Path) -> None:
    domain = tmp_path / "card_patch_lab"
    pkg = domain / "demo_lab" / "devices" / "my_robot"
    pkg.mkdir(parents=True)
    (domain / "pyproject.toml").write_text(
        '[project]\nname = "demo"\nversion = "0.1.0"\ndependencies = []\n',
        encoding="utf-8",
    )
    (pkg / "device.py").write_text(
        '''from unilabos.registry.decorators import device

@device(id="my_robot", model={"type": "package_moveit", "provider": "unilab_arm_cr5:build_moveit_model", "source_digest": "0"*64})
class MyRobotDevice:
    pass
''',
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
            "--apply",
            "--skip-check",
        ]
    )
    assert report["_exit"] == 0
    device_py = (pkg / "device.py").read_text(encoding="utf-8")
    assert "MyRobotArmCardMixin" in device_py
    assert (pkg / "my_robot_arm_card.py").is_file()
    assert report["card_ensure"]["created_or_patched"] is True


def test_catalog_auto_apply_writes_manifest(tmp_path: Path) -> None:
    domain = tmp_path / "demo-lab"
    pkg = domain / "demo_lab" / "devices" / "my_robot"
    pkg.mkdir(parents=True)
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
    (pkg / "device.py").write_text(
        '''
from unilabos.registry.decorators import device

@device(id="my_robot", model={"type": "package_moveit", "provider": "unilab_arm_cr5:build_moveit_model", "source_digest": "0"*64})
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
            "--apply",
            "--skip-check",
        ]
    )
    assert report["_exit"] == 0
    manifest = domain / "frontend" / "cards" / "my-robot-card" / "card.manifest.json"
    assert manifest.is_file()
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    assert payload["deviceTypes"] == ["community.demo_lab.my_robot"]
    assert payload["templateCard"] == "unilab_robot_template/frontend/cards/rail-mounted-arm-card"


def test_preview_catalog_dry_run_scaffold_plan(tmp_path: Path) -> None:
    domain = tmp_path / "demo-lab"
    (domain / "demo_lab" / "devices").mkdir(parents=True)
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
    report = _run(
        [
            "--domain",
            str(domain),
            "--device",
            "elite_preview",
            "--preview-catalog",
            "elite-cs66",
            "--with-preview-scaffold",
        ]
    )
    assert report["_exit"] == 0
    assert report["mode"] == "catalog-preview"
    assert report["model"]["type"] == "package_static"
    assert report["demo_index"] == "docs/demo/README.md"
    scaffold = report["preview_scaffold"]
    assert "demo_lab/devices/elite_preview/device.py" in scaffold["planned_files"]


def test_greenfield_defaults_without_explicit_catalog(tmp_path: Path) -> None:
    domain = tmp_path / "DefaultLab"
    report = _run(
        [
            "--new-domain",
            str(domain),
            "--device",
            "robot",
        ]
    )
    assert report["_exit"] == 0
    assert report["default_models"]["arm_catalog"] == "cr5"
    assert report["rail_scaffold"]["static_digest"] == (
        "b19c65c03a228077fc7466db2ae4fbc686a5eb7ffb36e8ec9806a8db6debe323"
    )
    assert report["rail_scaffold"]["rail_provider"] == (
        "unilab_rail_linear.static_layout:build_default_rail"
    )
    assert "devices/rail.py" in report["rail_scaffold"]["planned_files"][0]
    assert "card" in report
    assert "moveit_arm_card" in report


def test_greenfield_catalog_dry_run(tmp_path: Path) -> None:
    domain = tmp_path / "MyLab"
    report = _run(
        [
            "--new-domain",
            str(domain),
            "--device",
            "robot",
            "--rail-device",
            "rail",
            "--catalog",
            "cr5",
        ]
    )
    assert report["_exit"] == 0
    assert report["greenfield"] is True
    assert report["dry_run"] is True
    assert report["demo_reference"] == "docs/demo/minimal-rail-arm-catalog"
    assert "domain_scaffold" in report
    assert "rail_scaffold" in report
    assert "graph_scaffold" in report
    assert report["graph_scaffold"]["node_ids"] == ["rail", "robot"]
    assert report["card"]["changed"] is True
    assert report["moveit_arm_card"]["changed"] is True


def test_greenfield_skip_card(tmp_path: Path) -> None:
    domain = tmp_path / "NoCardLab"
    report = _run(
        [
            "--new-domain",
            str(domain),
            "--device",
            "robot",
            "--skip-card",
        ]
    )
    assert report["_exit"] == 0
    assert "card" not in report
    assert "moveit_arm_card" not in report


def test_greenfield_catalog_apply_writes_scaffold(tmp_path: Path) -> None:
    domain = tmp_path / "my_lab_greenfield"
    report = _run(
        [
            "--new-domain",
            str(domain),
            "--device",
            "robot",
            "--catalog",
            "cr5",
            "--apply",
            "--skip-check",
        ]
    )
    assert report["_exit"] == 0
    assert (domain / "pyproject.toml").is_file()
    assert "[tool.unilabos.startup]" in (domain / "pyproject.toml").read_text(encoding="utf-8")
    assert (domain / "deployment" / "local_config.py").is_file()
    assert (domain / "package.yaml").is_file()
    assert (domain / "my_lab_greenfield" / "workflows" / "__init__.py").is_file()
    env_local = domain / ".unilabos" / "environment.local.json"
    assert env_local.is_file()
    assert json.loads(env_local.read_text(encoding="utf-8"))["externalDevicesOnly"] is False
    assert (domain / "my_lab_greenfield" / "devices" / "rail.py").is_file()
    assert "unilab_rail_linear.static_layout:build_default_rail" in (
        domain / "my_lab_greenfield" / "devices" / "rail.py"
    ).read_text(encoding="utf-8")
    assert (domain / "my_lab_greenfield" / "devices" / "robot" / "device.py").is_file()
    device_py = (domain / "my_lab_greenfield" / "devices" / "robot" / "device.py").read_text(
        encoding="utf-8"
    )
    assert "RobotArmCardMixin" in device_py
    assert (domain / "my_lab_greenfield" / "devices" / "robot" / "robot_arm_card.py").is_file()
    arm_card_py = (
        domain / "my_lab_greenfield" / "devices" / "robot" / "robot_arm_card.py"
    ).read_text(encoding="utf-8")
    assert "NotImplementedError" not in arm_card_py
    assert "build_robot_arm_card_context" in arm_card_py
    assert (
        domain / "my_lab_greenfield" / "devices" / "robot" / "moveit_commissioning.py"
    ).is_file()
    assert (domain / "my_lab_greenfield" / "robot_cell" / "point_set_v3.py").is_file()
    assert (domain / "my_lab_greenfield" / "devices" / "rail_simulation.py").is_file()
    point_set = (
        domain
        / "deployment"
        / "robot_cell"
        / "assets"
        / "point_sets"
        / "my_lab_greenfield-rail-arm.v3.yaml"
    )
    assert point_set.is_file()
    rail_py = (domain / "my_lab_greenfield" / "devices" / "rail.py").read_text(
        encoding="utf-8"
    )
    assert "post_init" in rail_py
    assert "_moveit_split_binding" in device_py
    assert report.get("card_assessment", {}).get("commissioning_complete") is True
    manifest = domain / "frontend" / "cards" / "robot-card" / "card.manifest.json"
    assert manifest.is_file()
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    assert payload["templateCard"] == "unilab_robot_template/frontend/cards/rail-mounted-arm-card"
    graph = domain / "deployment" / "graphs" / "local-debug.json"
    assert graph.is_file()
    payload = json.loads(graph.read_text(encoding="utf-8"))
    nodes = {node["id"]: node for node in payload["nodes"]}
    assert nodes["robot"]["parent"] == "rail"
    assert "robot" in nodes["rail"]["children"]


def test_greenfield_catalog_checker_passes(tmp_path: Path) -> None:
    domain = tmp_path / "checker_lab"
    report = _run(
        [
            "--new-domain",
            str(domain),
            "--device",
            "robot",
            "--catalog",
            "cr5",
            "--apply",
        ]
    )
    assert report["_exit"] == 0
    checker = report.get("checker") or {}
    assert checker.get("exit_code") == 0, checker.get("payload")


def test_graph_merge_appends_missing_nodes(tmp_path: Path) -> None:
    domain = tmp_path / "merge-lab"
    pkg = domain / "merge_lab"
    pkg.mkdir(parents=True)
    (domain / "pyproject.toml").write_text(
        """
[project]
dependencies = []
[tool.setuptools.packages.find]
include = ["merge_lab*"]
""".strip()
        + "\n",
        encoding="utf-8",
    )
    graph_dir = domain / "deployment" / "graphs"
    graph_dir.mkdir(parents=True)
    (graph_dir / "local-debug.json").write_text(
        json.dumps({"nodes": [{"id": "other", "type": "device"}], "links": []}),
        encoding="utf-8",
    )
    report = _run(
        [
            "--domain",
            str(domain),
            "--device",
            "robot",
            "--catalog",
            "cr5",
            "--scaffold-graph",
            "--apply",
            "--skip-check",
        ]
    )
    assert report["_exit"] == 0
    payload = json.loads((graph_dir / "local-debug.json").read_text(encoding="utf-8"))
    node_ids = {node["id"] for node in payload["nodes"]}
    assert "other" in node_ids
    assert "rail" in node_ids
    assert "robot" in node_ids


def test_domain_owned_vendor_dry_run(tmp_path: Path) -> None:
    template_root = Path(__file__).resolve().parents[3]
    vendor_urdf = (
        template_root
        / "packages"
        / "unilab-arm-cr5"
        / "src"
        / "unilab_arm_cr5"
        / "models"
        / "cr5_robot.urdf"
    )
    domain = tmp_path / "owned_lab"
    report = _run(
        [
            "--new-domain",
            str(domain),
            "--device",
            "my_robot",
            "--domain-owned",
            "--vendor-urdf",
            str(vendor_urdf),
            "--apply",
            "--skip-check",
        ]
    )
    assert report["_exit"] == 0
    assert "vendor_assets" in report
    model_yaml = domain / "owned_lab" / "devices" / "my_robot" / "models" / "model.yaml"
    assert model_yaml.is_file()
    assert (domain / "owned_lab" / "devices" / "my_robot" / "moveit_model.py").is_file()


def test_preview_catalog_apply_writes_scaffold(tmp_path: Path) -> None:
    domain = tmp_path / "demo-lab"
    (domain / "demo_lab" / "devices").mkdir(parents=True)
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
    report = _run(
        [
            "--domain",
            str(domain),
            "--device",
            "elite_preview",
            "--preview-catalog",
            "elite-cs66",
            "--with-preview-scaffold",
            "--apply",
            "--skip-check",
        ]
    )
    assert report["_exit"] == 0
    device_py = domain / "demo_lab" / "devices" / "elite_preview" / "device.py"
    assert device_py.is_file()
    assert "package_static" in device_py.read_text(encoding="utf-8")
    assert "unilab-arm-elite-cs66" in (domain / "pyproject.toml").read_text(encoding="utf-8")
