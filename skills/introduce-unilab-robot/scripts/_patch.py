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
) -> str:
    model_text = pprint.pformat(model, width=100, sort_dicts=False)
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


def pip_install_editable(package_dir: Path) -> tuple[int, str]:
    command = [sys.executable, "-m", "pip", "install", "-e", str(package_dir)]
    proc = subprocess.run(command, capture_output=True, text=True, check=False)
    output = (proc.stdout or "") + (proc.stderr or "")
    return proc.returncode, output.strip()
