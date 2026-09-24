"""写 pyproject 依赖与 @device model 块。"""

from __future__ import annotations

import ast
import pprint
import re
import subprocess
import sys
from pathlib import Path
from typing import Any


def ensure_pyproject_dependencies(pyproject: Path, dependencies: list[str]) -> tuple[str, list[str]]:
    text = pyproject.read_text(encoding="utf-8")
    added: list[str] = []
    updated = text
    for dep in dependencies:
        name = dep.split(">=", 1)[0].split("==", 1)[0].strip().strip('"')
        if re.search(rf'["\']{re.escape(name)}[^"\']*["\']', updated):
            continue
        match = re.search(r"(dependencies\s*=\s*\[)([^\]]*)(\])", updated, re.S)
        if match is None:
            raise ValueError(f"{pyproject} 缺少 [project].dependencies 数组")
        block = match.group(2)
        insertion = f'{block.rstrip()}\n  "{dep}",\n'
        updated = updated[: match.start(2)] + insertion + updated[match.end(2) :]
        added.append(dep)
    return updated, added


def patch_model_block(source: str, model: dict[str, Any]) -> tuple[str, bool]:
    tree = ast.parse(source)
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
            for keyword in decorator.keywords:
                if keyword.arg != "model":
                    continue
                segment = ast.get_source_segment(source, keyword.value)
                if segment is None:
                    raise ValueError("无法定位 model= 源码片段")
                rendered = pprint.pformat(model, width=100, sort_dicts=False)
                updated = source.replace(segment, rendered, 1)
                return updated, updated != source
    return source, False


def scaffold_device_py(
    *,
    device_id: str,
    class_name: str,
    model: dict[str, Any],
    with_card: bool = False,
    domain_pkg: str | None = None,
    card_class_prefix: str | None = None,
) -> str:
    model_text = pprint.pformat(model, width=100, sort_dicts=False)
    if with_card and domain_pkg and card_class_prefix:
        return f'''"""由 introduce-unilab-robot 自动生成；含 MoveIt 机械臂卡片 mixin。"""
from __future__ import annotations

from typing import Any

from unilabos.registry.decorators import device, not_action

from {domain_pkg}.devices.{device_id}.{device_id}_arm_card import {card_class_prefix}ArmCardMixin


@device(
    id="{device_id}",
    category=["robot"],
    description="TODO: 补充设备说明",
    model={model_text},
)
class {class_name}({card_class_prefix}ArmCardMixin):
    def __init__(self, device_id: str | None = None, config: dict | None = None, **kwargs: object) -> None:
        self.device_id = device_id or "{device_id}"
        self.config = dict(config or {{}})
        self.config.update(kwargs)
        self._moveit_split_binding: Any = None
        self._moveit_client: Any = None

    @not_action
    def post_init(self, ros_node: Any) -> None:
        backend = str(self.config.get("standard_execution_backend", "")).strip()
        if backend not in {{"moveit", "moveit_sim"}}:
            return
        from {domain_pkg}.devices.{device_id}.moveit_commissioning import (
            build_robot_moveit_binding,
        )

        model = type(self)._device_registry_meta["model"]
        binding, client = build_robot_moveit_binding(
            device_id=self.device_id,
            ros_node=ros_node,
            model_config=model,
            node_config=self.config,
        )
        self._moveit_split_binding = binding
        self._moveit_client = client
'''
    return f'''"""由 introduce-unilab-robot 自动生成；请补充设备动作。"""
from __future__ import annotations

from unilabos.registry.decorators import device


@device(
    id="{device_id}",
    category=["robot"],
    description="TODO: 补充设备说明",
    model={model_text},
)
class {class_name}:
    def __init__(self, device_id: str | None = None, config: dict | None = None, **kwargs: object) -> None:
        self.device_id = device_id or "{device_id}"
        self.config = dict(config or {{}})
'''


def patch_device_card_inheritance(
    source: str,
    *,
    domain_pkg: str,
    device_id: str,
    card_class_prefix: str,
) -> tuple[str, bool]:
    mixin_name = f"{card_class_prefix}ArmCardMixin"
    if mixin_name in source:
        return source, False
    import_line = (
        f"from {domain_pkg}.devices.{device_id}.{device_id}_arm_card import {mixin_name}\n"
    )
    if import_line not in source:
        marker = "from unilabos.registry.decorators import device\n"
        if marker not in source:
            return source, False
        source = source.replace(marker, marker + "\n" + import_line, 1)
    pattern = re.compile(
        rf"(class\s+\w+\s*)\(([^)]*)\)\s*:",
        re.MULTILINE,
    )
    match = pattern.search(source)
    if match is None:
        pattern = re.compile(r"(class\s+\w+\s*):", re.MULTILINE)
        match = pattern.search(source)
        if match is None:
            return source, False
        updated = source[: match.end()] + f"({mixin_name})" + source[match.end() :]
        return updated, updated != source
    bases = match.group(2).strip()
    if mixin_name in bases:
        return source, False
    if bases:
        replacement = f"{match.group(1)}({bases}, {mixin_name}):"
    else:
        replacement = f"{match.group(1)}({mixin_name}):"
    updated = source[: match.start()] + replacement + source[match.end() :]
    return updated, updated != source


def pip_install_editable(package_dir: Path) -> tuple[int, str]:
    command = [sys.executable, "-m", "pip", "install", "-e", str(package_dir)]
    proc = subprocess.run(command, capture_output=True, text=True, check=False)
    output = (proc.stdout or "") + (proc.stderr or "")
    return proc.returncode, output.strip()
