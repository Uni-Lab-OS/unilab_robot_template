"""不执行 device 包 __init__.py 的 robot_module 加载。"""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path
from typing import Any


def load_robot_module(*, domain: Path, domain_pkg: str, device_dir: Path, module_name: str) -> Any:
    """加载 robot_module.py 并满足相对 import，不触发 SzlabMixerRobotDevice。"""

    domain = domain.resolve()
    root = str(domain)
    if root not in sys.path:
        sys.path.insert(0, root)

    pkg_path = domain / domain_pkg.replace(".", "/")
    devices_path = pkg_path / "devices"
    device_id = device_dir.name

    package_paths = {
        domain_pkg: pkg_path,
        f"{domain_pkg}.devices": devices_path,
        f"{domain_pkg}.devices.{device_id}": device_dir,
    }
    for name, path in package_paths.items():
        if name not in sys.modules:
            module = types.ModuleType(name)
            module.__path__ = [str(path)]  # type: ignore[attr-defined]
            sys.modules[name] = module

    file_path = device_dir / "robot_module.py"
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"无法加载 {file_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module
