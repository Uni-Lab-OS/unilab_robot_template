"""领域仓机械臂 device 扫描（只读 AST / 文本）。"""

from __future__ import annotations

import ast
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

_DOMAIN_ARM_PROVIDER = re.compile(
    r"^(?P<module>[A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z_][A-Za-z0-9_]*)*):build_moveit_model$"
)
_RUNTIME_ASSEMBLY_NAMES = (
    "moveit_runtime_assembly.py",
    "robot_runtime_assembly.py",
)
_ARM_MODULE_REF = re.compile(
    r'arm\s*=\s*_ModuleRef\s*\(\s*'
    r'"([^"]+)"\s*,\s*'
    r'"([^"]+)"\s*,\s*'
    r"frozenset\([^)]*\)\s*,\s*"
    r'"([^"]+)"\s*,?\s*\)',
    re.DOTALL,
)
_ROBOT_MODULE_SUFFIX = ".robot_module"


@dataclass(frozen=True)
class ArmDeviceRecord:
    domain: Path
    device_id: str
    device_dir: Path
    device_py: Path
    provider: str
    provider_module: str
    domain_pkg: str
    robot_module_path: str


def find_arm_device(domain: Path, device_id: str) -> ArmDeviceRecord | None:
    """按 device id 定位 package_moveit 机械臂。"""

    domain = domain.resolve()
    for device_py in domain.rglob("device.py"):
        if "tests" in device_py.parts or ".unilabos" in device_py.parts:
            continue
        if device_py.parent.name != device_id:
            continue
        model, declared_id = _device_decorator_payload(device_py)
        if model is None or model.get("type") != "package_moveit":
            continue
        resolved_id = declared_id or device_py.parent.name
        if resolved_id != device_id:
            continue
        provider = str(model.get("provider") or "").strip()
        match = _DOMAIN_ARM_PROVIDER.fullmatch(provider)
        if match is None:
            return None
        provider_module = str(match.group("module"))
        if ".devices." not in provider_module:
            return None
        domain_pkg = provider_module.split(".devices.", 1)[0]
        robot_module_path = provider_module.rsplit(".", 1)[0] + _ROBOT_MODULE_SUFFIX
        return ArmDeviceRecord(
            domain=domain,
            device_id=resolved_id,
            device_dir=device_py.parent,
            device_py=device_py,
            provider=provider,
            provider_module=provider_module,
            domain_pkg=domain_pkg,
            robot_module_path=robot_module_path,
        )
    return None


def list_arm_devices(domain: Path) -> list[ArmDeviceRecord]:
    """列出领域仓全部 package_moveit 机械臂。"""

    domain = domain.resolve()
    seen: set[str] = set()
    records: list[ArmDeviceRecord] = []
    for device_py in domain.rglob("device.py"):
        if "tests" in device_py.parts or ".unilabos" in device_py.parts:
            continue
        device_id = device_py.parent.name
        if device_id in seen:
            continue
        record = find_arm_device(domain, device_id)
        if record is None:
            continue
        seen.add(device_id)
        records.append(record)
    return sorted(records, key=lambda item: item.device_id)


def runtime_assembly_files(device_dir: Path) -> list[Path]:
    """返回 device 目录及子目录内的 runtime assembly 文件。"""

    found: list[Path] = []
    for name in _RUNTIME_ASSEMBLY_NAMES:
        path = device_dir / name
        if path.is_file():
            found.append(path)
    for path in sorted(device_dir.rglob("*runtime_assembly*.py")):
        if "tests" in path.parts:
            continue
        if path not in found:
            found.append(path)
    return found


def read_manifest_arm_module(text: str) -> str | None:
    """从 runtime assembly 源码读出 arm _ModuleRef 的 python_package。"""

    match = _ARM_MODULE_REF.search(text)
    if match is None:
        return None
    return match.group(3)


def read_manifest_distribution_version(text: str) -> tuple[str | None, str | None]:
    """读出 arm _ModuleRef 的 distribution 与 version。"""

    match = _ARM_MODULE_REF.search(text)
    if match is None:
        return None, None
    return match.group(1), match.group(2)


def manifest_points_to_moveit_model(text: str, provider_module: str) -> bool:
    """manifest 是否仍把 arm 指到 moveit_model。"""

    arm_module = read_manifest_arm_module(text)
    if arm_module is None:
        return False
    if arm_module.endswith(".moveit_model"):
        return True
    return arm_module == provider_module


def patch_manifest_arm_module(
    text: str,
    *,
    provider_module: str,
    robot_module_path: str,
) -> tuple[str, bool]:
    """把 arm _ModuleRef 的 python_package 改成 robot_module。"""

    match = _ARM_MODULE_REF.search(text)
    if match is None:
        return text, False
    current = match.group(3)
    if current == robot_module_path:
        return text, False
    if not (
        current.endswith(".moveit_model")
        or current == provider_module
        or current.startswith("unilab_arm_")
    ):
        return text, False
    updated = text[: match.start(3)] + robot_module_path + text[match.end(3) :]
    return updated, True


def _device_decorator_payload(device_py: Path) -> tuple[dict[str, Any] | None, str]:
    tree = ast.parse(device_py.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.ClassDef):
            continue
        for decorator in node.decorator_list:
            if not isinstance(decorator, ast.Call):
                continue
            func = decorator.func
            name = func.id if isinstance(func, ast.Name) else getattr(func, "attr", "")
            if name != "device":
                continue
            model: dict[str, Any] | None = None
            device_id = ""
            for keyword in decorator.keywords:
                if keyword.arg == "model":
                    try:
                        value = ast.literal_eval(keyword.value)
                    except (ValueError, TypeError):
                        return None, ""
                    if isinstance(value, dict):
                        model = value
                if keyword.arg == "id":
                    try:
                        device_id = str(ast.literal_eval(keyword.value) or "")
                    except (ValueError, TypeError):
                        device_id = ""
            return model, device_id
    return None, ""
