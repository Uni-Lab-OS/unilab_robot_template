"""领域仓布局探测与 device 文件定位。"""

from __future__ import annotations

import ast
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class DeviceTarget:
    domain: Path
    domain_pkg: str
    device_id: str
    device_py: Path
    class_name: str | None
    existing_model: dict[str, Any] | None


def detect_domain_package(domain: Path) -> str:
    domain = domain.resolve()
    pyproject = domain / "pyproject.toml"
    if pyproject.is_file():
        text = pyproject.read_text(encoding="utf-8")
        match = re.search(
            r'include\s*=\s*\[\s*"([A-Za-z_][A-Za-z0-9_]*)',
            text,
        )
        if match:
            return match.group(1).rstrip("*")
    for child in sorted(domain.iterdir()):
        if child.is_dir() and (child / "devices").is_dir() and not child.name.startswith("."):
            return child.name
    raise ValueError(f"无法从 {domain} 推断 Python 包名（请检查 pyproject.toml）")


def default_device_py(domain: Path, domain_pkg: str, device_id: str) -> Path:
    return domain / domain_pkg / "devices" / device_id / "device.py"


def _class_name(device_id: str) -> str:
    parts = [part for part in re.split(r"[^A-Za-z0-9]+", device_id) if part]
    if not parts:
        return "RobotDevice"
    return "".join(part[:1].upper() + part[1:] for part in parts) + "Device"


def _device_decorator_payload(decorator: ast.AST) -> tuple[dict[str, Any] | None, str, str | None]:
    if not isinstance(decorator, ast.Call):
        return None, "", None
    func = decorator.func
    name = func.id if isinstance(func, ast.Name) else getattr(func, "attr", "")
    if name != "device":
        return None, "", None
    model: dict[str, Any] | None = None
    device_id = ""
    for keyword in decorator.keywords:
        if keyword.arg == "model":
            try:
                value = ast.literal_eval(keyword.value)
            except (ValueError, TypeError):
                return None, "", None
            if isinstance(value, dict):
                model = value
        if keyword.arg == "id":
            try:
                device_id = str(ast.literal_eval(keyword.value) or "")
            except (ValueError, TypeError):
                device_id = ""
    return model, device_id, None


def locate_device(domain: Path, device_id: str, domain_pkg: str | None = None) -> DeviceTarget:
    domain = domain.resolve()
    pkg = domain_pkg or detect_domain_package(domain)
    found: DeviceTarget | None = None
    for path in domain.rglob("*.py"):
        if "tests" in path.parts or ".unilabos" in path.parts:
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"))
        except (OSError, SyntaxError):
            continue
        for node in ast.walk(tree):
            if not isinstance(node, ast.ClassDef):
                continue
            for decorator in node.decorator_list:
                model, declared_id, _ = _device_decorator_payload(decorator)
                if model is None:
                    continue
                resolved_id = declared_id or node.name
                if resolved_id != device_id:
                    continue
                found = DeviceTarget(
                    domain=domain,
                    domain_pkg=pkg,
                    device_id=resolved_id,
                    device_py=path,
                    class_name=node.name,
                    existing_model=model,
                )
                break
    if found is not None:
        return found
    return DeviceTarget(
        domain=domain,
        domain_pkg=pkg,
        device_id=device_id,
        device_py=default_device_py(domain, pkg, device_id),
        class_name=_class_name(device_id),
        existing_model=None,
    )
