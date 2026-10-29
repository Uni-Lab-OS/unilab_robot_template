#!/usr/bin/env python3
"""检查领域仓是否按 pTLC 方式消费机械臂 / 导轨型号包。

参数：``--domain`` 是领域仓根目录。返回：全部不变量成立时退出 0，否则退出 1
并在 stdout 打印 JSON 报告。异常：路径不可读时退出 2。
安全：只读 AST 与 JSON，不导入领域包，不启动 OS。
"""

from __future__ import annotations

import argparse
import ast
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

_DIGEST = re.compile(r"^[0-9a-f]{64}$")
_ARM_PROVIDER = re.compile(
    r"^unilab_arm_(?P<slug>[a-z0-9]+):build_moveit_model$"
)
_RAIL_JOINT_PROVIDER = "unilab_rail_linear:build_kinematic_model"
_RAIL_BACKEND_FORBIDDEN = frozenset({"moveit", "moveit_sim"})
_GRAPH_SKIP_SUBSTRINGS = (" copy",)
_RAIL_DIGEST_HINT = "9ec7d9833f46c26e02e08f06aecd12495e4ab6753ebd1e47a967f7bf885bf83d"


@dataclass
class DeviceModel:
    """从 ``@device`` 装饰器抽出的模型配置。"""

    path: str
    device_id: str
    class_name: str
    model: dict[str, Any]


@dataclass
class Report:
    """一次领域仓扫描的可机读结果。"""

    domain: str
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    arms: list[str] = field(default_factory=list)
    rails: list[str] = field(default_factory=list)
    graphs: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        """没有错误时才算通过。"""

        return not self.errors


def main(argv: list[str] | None = None) -> int:
    """解析参数、扫描领域仓、打印 JSON。"""

    parser = argparse.ArgumentParser(
        description="按 pTLC 合同检查领域仓机械臂/导轨装配"
    )
    parser.add_argument("--domain", required=True, type=Path)
    args = parser.parse_args(argv)
    domain = args.domain.resolve()
    if not domain.is_dir():
        print(json.dumps({"ok": False, "error": f"领域仓不存在: {domain}"}, ensure_ascii=False))
        return 2
    report = check_domain(domain)
    print(json.dumps(report_to_dict(report), ensure_ascii=False, indent=2))
    return 0 if report.ok else 1


def check_domain(domain: Path) -> Report:
    """扫描 ``@device`` 与物理图，收集装配违规。"""

    report = Report(domain=str(domain))
    devices = _collect_device_models(domain)
    arms = [item for item in devices if item.model.get("type") == "package_moveit"]
    rails = [
        item
        for item in devices
        if item.model.get("type") == "package_static"
        and item.model.get("joint_state_provider")
    ]
    report.arms = [item.device_id for item in arms]
    report.rails = [item.device_id for item in rails]
    if not arms:
        report.errors.append("未找到 type=package_moveit 的机械臂 @device")
    for arm in arms:
        _check_arm_model(arm, report)
    for rail in rails:
        _check_rail_model(rail, report)
    _check_static_root_links(domain, report)
    _check_legacy_seventh_axis_files(domain, report)
    graphs = _graph_files(domain)
    report.graphs = [str(path.relative_to(domain)).replace("\\", "/") for path in graphs]
    if not graphs:
        report.errors.append("deployment/graphs 下没有可检查的物理图 JSON")
    for graph_path in graphs:
        _check_graph(graph_path, arms, rails, report)
    return report


def report_to_dict(report: Report) -> dict[str, Any]:
    """把报告变成稳定 JSON 对象。"""

    return {
        "ok": report.ok,
        "domain": report.domain,
        "arms": report.arms,
        "rails": report.rails,
        "graphs": report.graphs,
        "errors": report.errors,
        "warnings": report.warnings,
    }


def _collect_device_models(domain: Path) -> list[DeviceModel]:
    """用 AST 读取全部 ``@device(model=...)`` 字面量。"""

    found: list[DeviceModel] = []
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
                model, device_id = _device_decorator_payload(decorator)
                if model is None:
                    continue
                found.append(
                    DeviceModel(
                        path=str(path.relative_to(domain)).replace("\\", "/"),
                        device_id=device_id or node.name,
                        class_name=node.name,
                        model=model,
                    )
                )
    return found


def _device_decorator_payload(
    decorator: ast.AST,
) -> tuple[dict[str, Any] | None, str]:
    """抽出 ``@device`` 的 model 字典和 id。"""

    if not isinstance(decorator, ast.Call):
        return None, ""
    func = decorator.func
    name = ""
    if isinstance(func, ast.Name):
        name = func.id
    elif isinstance(func, ast.Attribute):
        name = func.attr
    if name != "device":
        return None, ""
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
                raw = ast.literal_eval(keyword.value)
            except (ValueError, TypeError):
                raw = ""
            device_id = str(raw or "")
    return model, device_id


def _check_arm_model(arm: DeviceModel, report: Report) -> None:
    """机械臂模型必须是锁定 digest 的六轴 package_moveit。"""

    model = arm.model
    prefix = f"{arm.path} @device {arm.device_id}"
    if model.get("type") != "package_moveit":
        report.errors.append(f"{prefix}: type 必须是 package_moveit")
    provider = str(model.get("provider") or "")
    if _ARM_PROVIDER.fullmatch(provider) is None:
        report.errors.append(
            f"{prefix}: provider 必须是 unilab_arm_<slug>:build_moveit_model，实际 {provider!r}"
        )
    digest = str(model.get("source_digest") or "").lower()
    if _DIGEST.fullmatch(digest) is None:
        report.errors.append(f"{prefix}: source_digest 必须是 SHA-256")
    dumped = json.dumps(model)
    if "arm_base_joint" in dumped:
        report.errors.append(f"{prefix}: 模型配置禁止出现 arm_base_joint")
    if str(model.get("type") or "") in {"xacro", "workspace_xacro"} or model.get(
        "format"
    ) == "xacro":
        report.errors.append(f"{prefix}: 禁止 xacro / workspace_xacro")
    if model.get("format") == "urdf" and not str(model.get("entry") or "").strip():
        report.warnings.append(
            f"{prefix}: format=urdf 但没有本地 entry；工作区物料目录会跳过或失败"
        )


def _check_rail_model(rail: DeviceModel, report: Report) -> None:
    """导轨必须是静态外壳加独立关节 Provider。"""

    model = rail.model
    prefix = f"{rail.path} @device {rail.device_id}"
    if model.get("type") != "package_static":
        report.errors.append(f"{prefix}: 带关节的导轨 type 必须是 package_static")
    if str(model.get("joint_state_provider") or "") != _RAIL_JOINT_PROVIDER:
        report.errors.append(
            f"{prefix}: joint_state_provider 必须是 {_RAIL_JOINT_PROVIDER}"
        )
    digest = str(model.get("joint_state_source_digest") or "").lower()
    if _DIGEST.fullmatch(digest) is None:
        report.errors.append(f"{prefix}: joint_state_source_digest 必须是 SHA-256")
    elif digest != _RAIL_DIGEST_HINT:
        report.warnings.append(
            f"{prefix}: joint_state_source_digest 与当前 unilab-rail-linear 锁定值不同"
        )
    if "arm_base_joint" in json.dumps(model):
        report.errors.append(f"{prefix}: 导轨模型禁止出现 arm_base_joint")


def _check_static_root_links(domain: Path, report: Report) -> None:
    """静态外壳根不得使用运动学 ``rail_base`` 名。"""

    pattern = re.compile(
        r'root_link\s*=\s*f?["\']\{?member_id\}?_rail_base["\']'
        r'|root_link\s*=\s*f["\']\{member_id\}_rail_base["\']'
    )
    for path in domain.rglob("*.py"):
        if "tests" in path.parts or ".unilabos" in path.parts:
            continue
        text = path.read_text(encoding="utf-8")
        if "_rail_base" not in text:
            continue
        if pattern.search(text) or 'root_link = f"{member_id}_rail_base"' in text:
            rel = str(path.relative_to(domain)).replace("\\", "/")
            report.errors.append(
                f"{rel}: 静态 root_link 禁止使用 {{member_id}}_rail_base，应使用 {{member_id}}_base_link"
            )


def _check_legacy_seventh_axis_files(domain: Path, report: Report) -> None:
    """已切 package_moveit 后，残留第七轴资产必须标出来，禁止再被抄回。"""

    if not report.arms:
        return
    for path in domain.rglob("*"):
        if not path.is_file():
            continue
        if "tests" in path.parts or ".unilabos" in path.parts:
            continue
        if path.suffix.lower() not in {".py", ".json", ".yaml", ".yml", ".xacro", ".urdf", ".srdf"}:
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            continue
        if "arm_base_joint" not in text:
            continue
        rel = str(path.relative_to(domain)).replace("\\", "/")
        report.warnings.append(
            f"{rel}: 残留 arm_base_joint；package_moveit 路径禁止再引用这份第七轴资产"
        )


def _graph_files(domain: Path) -> list[Path]:
    """返回领域仓生产物理图。"""

    root = domain / "deployment" / "graphs"
    if not root.is_dir():
        return []
    files: list[Path] = []
    for path in sorted(root.glob("*.json")):
        if any(token in path.name for token in _GRAPH_SKIP_SUBSTRINGS):
            continue
        files.append(path)
    return files


def _check_graph(
    graph_path: Path,
    arms: list[DeviceModel],
    rails: list[DeviceModel],
    report: Report,
) -> None:
    """一张物理图必须把导轨当父设备，且后端正交。"""

    prefix = graph_path.name
    try:
        payload = json.loads(graph_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        report.errors.append(f"{prefix}: 不是合法 JSON ({error})")
        return
    dumped = json.dumps(payload)
    if "arm_base_joint" in dumped:
        report.errors.append(f"{prefix}: 物理图禁止出现 arm_base_joint")
    nodes = {
        str(node.get("id") or ""): node
        for node in payload.get("nodes") or []
        if isinstance(node, dict)
    }
    arm_ids = {item.device_id for item in arms}
    rail_ids = {item.device_id for item in rails}
    present_arms = [node_id for node_id in arm_ids if node_id in nodes]
    present_rails = [node_id for node_id in rail_ids if node_id in nodes]
    if arm_ids and rail_ids and present_arms and not present_rails:
        report.errors.append(f"{prefix}: 有机械臂节点但缺少对应导轨节点")
    for arm_id in present_arms:
        arm_node = nodes[arm_id]
        if present_rails:
            parent = str(arm_node.get("parent") or "")
            if parent not in rail_ids:
                report.errors.append(
                    f"{prefix} {arm_id}: parent 必须是导轨设备 id，实际 {parent!r}"
                )
        _check_yaw_not_stacked(arm_id, arm_node, arms, report, prefix)
    for rail_id in present_rails:
        rail_node = nodes[rail_id]
        backend = _backend(rail_node)
        if backend in _RAIL_BACKEND_FORBIDDEN:
            report.errors.append(
                f"{prefix} {rail_id}: 导轨禁止使用 MoveIt 执行后端"
            )
        children = rail_node.get("children") or []
        if present_arms and not any(child in arm_ids for child in children):
            report.errors.append(
                f"{prefix} {rail_id}: children 必须包含机械臂设备 id"
            )


def _check_yaw_not_stacked(
    arm_id: str,
    arm_node: dict[str, Any],
    arms: list[DeviceModel],
    report: Report,
    prefix: str,
) -> None:
    """图上 180° 与 mount_yaw_deg 不得叠加。"""

    model = next((item.model for item in arms if item.device_id == arm_id), {})
    mount_yaw = model.get("mount_yaw_deg")
    if mount_yaw is None:
        config = arm_node.get("config")
        if isinstance(config, dict):
            mount_yaw = config.get("mount_yaw_deg")
    try:
        yaw_fixed = abs(float(mount_yaw)) if mount_yaw is not None else 0.0
    except (TypeError, ValueError):
        report.errors.append(f"{prefix} {arm_id}: mount_yaw_deg 必须是有限角度")
        return
    rotation_z = _graph_rotation_z_deg(arm_node)
    if yaw_fixed >= 1.0 and abs(rotation_z) >= 1.0:
        report.errors.append(
            f"{prefix} {arm_id}: 同时存在 mount_yaw_deg={yaw_fixed} 与图 rotation.z={rotation_z}，安装偏航会叠加"
        )


def _graph_rotation_z_deg(node: dict[str, Any]) -> float:
    """读取物理图局部旋转 Z（度）。"""

    container = node.get("pose") if isinstance(node.get("pose"), dict) else node.get("position")
    if not isinstance(container, dict):
        return 0.0
    rotation = container.get("rotation")
    if not isinstance(rotation, dict):
        return 0.0
    try:
        return float(rotation.get("z") or 0.0)
    except (TypeError, ValueError):
        return 0.0


def _backend(node: dict[str, Any]) -> str:
    """读取物理图显式执行后端。"""

    config = node.get("config")
    if not isinstance(config, dict):
        return ""
    raw = config.get("standard_execution_backend") or config.get("execution_backend")
    return str(raw or "").strip()


if __name__ == "__main__":
    sys.exit(main())
