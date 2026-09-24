"""最小导轨+机械臂领域包 scaffold。"""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any


BASE_DEPENDENCIES = (
    "PyYAML>=6.0",
    "unilab-rail-linear>=0.1,<0.2",
    "unilab-robot-runtime>=0.1,<0.2",
)

DOMAIN_OWNED_EXTRA = (
    "unilab-robot-contracts>=0.1,<0.2",
    "unilab-robot-model-kit>=0.1,<0.2",
)

DEFAULT_GRAPH_REL = "deployment/graphs/local-debug.json"
DEFAULT_LOCAL_CONFIG_REL = "deployment/local_config.py"


def infer_domain_pkg(domain_path: Path) -> str:
    """从目录名推导 Python 包名（MyLab -> my_lab）。"""

    name = domain_path.name.strip()
    if not name:
        raise ValueError("领域仓路径缺少目录名")
    parts = re.split(r"[^A-Za-z0-9]+", name)
    tokens = [part.lower() for part in parts if part]
    if not tokens:
        raise ValueError(f"无法从 {domain_path.name} 推导 domain_pkg")
    return "_".join(tokens)


def pyproject_text(
    *,
    domain_pkg: str,
    project_name: str,
    domain_owned: bool,
    graph_rel: str = DEFAULT_GRAPH_REL,
) -> str:
    deps = list(BASE_DEPENDENCIES)
    if domain_owned:
        deps.extend(DOMAIN_OWNED_EXTRA)
    dep_lines = "\n".join(f'  "{item}",' for item in deps)
    normalized_name = project_name.replace("_", "-")
    return f"""[build-system]
requires = ["setuptools>=68", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "{normalized_name}"
version = "0.1.0"
description = "Minimal rail + arm domain package scaffolded by introduce-unilab-robot"
requires-python = ">=3.11"
dependencies = [
{dep_lines}
]

[tool.setuptools.packages.find]
include = ["{domain_pkg}*"]

[tool.unilabos.startup]
graph = "{graph_rel}"

[tool.pytest.ini_options]
addopts = "-q"
testpaths = ["tests"]
"""


def local_config_text() -> str:
    return '''"""Loopback-only local debug configuration."""


class BasicConfig:
    ak = ""
    sk = ""
    disable_browser = True
    no_update_feedback = True
    log_level = "INFO"


class WSConfig:
    reconnect_interval = 5
    max_reconnect_attempts = 999
    ws_ping_interval = 5
    ws_ping_timeout = 8


class MoveItConfig:
    plan_retry_attempts = 10
    num_planning_attempts = 10
    allowed_planning_time = 10.0
'''


def package_yaml_text(*, domain_pkg: str) -> str:
    return f"""package:
  name: {domain_pkg}

workflows: []
"""


def environment_local_json_text(*, graph_rel: str) -> str:
    payload = {
        "backendUrl": None,
        "domainMode": "local",
        "externalDevicesOnly": False,
        "graphPath": graph_rel,
        "runtimeMode": "normal",
        "schedulerUrl": None,
        "schemaVersion": 1,
    }
    return json.dumps(payload, ensure_ascii=False, indent=2) + "\n"


def registry_smoke_test() -> str:
    return '''"""最小领域包 registry smoke。"""

from __future__ import annotations


def test_package_importable() -> None:
    import importlib

    pkg = importlib.import_module("__DOMAIN_PKG__")
    assert pkg is not None
'''


def scaffold_domain_package(
    domain: Path,
    *,
    domain_pkg: str,
    domain_owned: bool,
    graph_rel: str = DEFAULT_GRAPH_REL,
    apply: bool,
) -> dict[str, Any]:
    """创建 pyproject、包根、Workbench 启动所需 deployment / package.yaml / workflows。"""

    domain = domain.resolve()
    pkg_root = domain / domain_pkg
    planned: list[str] = []
    created: list[str] = []

    targets = {
        domain / "pyproject.toml": pyproject_text(
            domain_pkg=domain_pkg,
            project_name=domain_pkg,
            domain_owned=domain_owned,
            graph_rel=graph_rel,
        ),
        pkg_root / "__init__.py": f'"""{domain_pkg} minimal rail+arm domain package."""\n',
        domain / "tests" / "test_registry_smoke.py": registry_smoke_test().replace(
            "__DOMAIN_PKG__", domain_pkg
        ),
        domain / DEFAULT_LOCAL_CONFIG_REL: local_config_text(),
        domain / "package.yaml": package_yaml_text(domain_pkg=domain_pkg),
        pkg_root / "workflows" / "__init__.py": '"""领域工作流（Workflow）源码目录。"""\n',
        domain / ".unilabos" / "environment.local.json": environment_local_json_text(
            graph_rel=graph_rel
        ),
    }

    for path, content in targets.items():
        rel = str(path.relative_to(domain)).replace("\\", "/")
        planned.append(rel)
        if path.is_file():
            continue
        if apply:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content if content.endswith("\n") else content + "\n", encoding="utf-8")
            created.append(rel)

    return {
        "domain_pkg": domain_pkg,
        "graph_rel": graph_rel,
        "planned_files": planned,
        "created_files": created,
        "changed": bool(created),
        "workbench_files": [
            DEFAULT_LOCAL_CONFIG_REL,
            "package.yaml",
            f"{domain_pkg}/workflows/__init__.py",
            ".unilabos/environment.local.json",
        ],
    }
