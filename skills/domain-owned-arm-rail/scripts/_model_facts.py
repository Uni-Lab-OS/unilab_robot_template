"""从 moveit_model.py 与 models/model.yaml 读取当前机械臂设备事实（只绑 device_id）。"""

from __future__ import annotations

import ast
import re
from dataclasses import dataclass
from pathlib import Path

import yaml

_ARM_ROLE_SUFFIX = "arm"
_FLANGE = re.compile(r"""flange_frame\s*=\s*['"]([^'"]+)['"]""")
_MODEL_DESCRIPTOR_REF = re.compile(
    r"""model_ref\s*=\s*(?:\(\s*)?["']([^"']+)["']""",
    re.MULTILINE,
)
_PLANNING_GROUP = re.compile(r"""planning_group\s*=\s*["']([^'"]+)["']""")
_CANONICAL_JOINTS = re.compile(
    r"""_CANONICAL_JOINT_NAMES\s*=\s*\((?P<body>[^)]*)\)""",
    re.DOTALL,
)
_JOINT_NAMES_BLOCK = re.compile(
    r"""_JOINT_NAMES\s*=\s*\{(?P<body>[^}]*)\}""",
    re.DOTALL,
)


@dataclass(frozen=True, slots=True)
class ArmModelFacts:
    device_id: str
    flange_frame: str
    distribution: str
    arm_endpoint_suffix: str
    canonical_joint_names: tuple[str, ...]
    moveit_group: str
    arm_endpoint: str
    model_yaml_path: Path | None


def expected_arm_endpoint(device_id: str) -> str:
    normalized = str(device_id).strip()
    if not normalized:
        raise ValueError("device_id 不能为空")
    return f"moveit:{normalized}:{_ARM_ROLE_SUFFIX}"


def expected_moveit_group(device_id: str) -> str:
    normalized = str(device_id).strip()
    if not normalized:
        raise ValueError("device_id 不能为空")
    return f"{normalized}_{_ARM_ROLE_SUFFIX}"


def _parse_canonical_joint_names(text: str) -> tuple[str, ...]:
    match = _CANONICAL_JOINTS.search(text)
    if match is None:
        return ()
    body = match.group("body")
    try:
        parsed = ast.literal_eval(f"({body})")
    except (SyntaxError, ValueError):
        return ()
    if not isinstance(parsed, tuple):
        return ()
    return tuple(str(value) for value in parsed)


def _parse_joint_name_values(text: str) -> tuple[str, ...]:
    match = _JOINT_NAMES_BLOCK.search(text)
    if match is None:
        return ()
    body = "{" + match.group("body") + "}"
    try:
        parsed = ast.literal_eval(body)
    except (SyntaxError, ValueError):
        return ()
    if not isinstance(parsed, dict):
        return ()
    arm_joints = [
        str(value)
        for value in parsed.values()
        if re.fullmatch(r"[A-Za-z0-9_]+_joint_[1-6]", str(value))
    ]
    return tuple(sorted(arm_joints, key=lambda name: int(name.rsplit("_", 1)[-1])))


def _read_model_yaml(model_yaml_path: Path) -> tuple[tuple[str, ...], str]:
    if not model_yaml_path.is_file():
        return (), ""
    data = yaml.safe_load(model_yaml_path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        return (), ""
    joints = tuple(str(value) for value in data.get("kinematic_joints", ()))
    moveit_group = str(data.get("moveit_group") or "").strip()
    return joints, moveit_group


def expected_domain_model_ref(domain_pkg: str, device_id: str) -> str:
    """PointSet / MODEL_DESCRIPTOR 共用的 arm 设备身份字符串。"""

    normalized_pkg = str(domain_pkg).strip()
    normalized_device = str(device_id).strip()
    if not normalized_pkg or not normalized_device:
        raise ValueError("domain_pkg 与 device_id 不能为空")
    return f"package://{normalized_pkg}/devices/{normalized_device}/models/model.yaml"


def read_moveit_model_descriptor_ref(moveit_model_path: Path) -> str | None:
    """读取 moveit_model.py 中 MODEL_DESCRIPTOR.model_ref。"""

    if not moveit_model_path.is_file():
        return None
    match = _MODEL_DESCRIPTOR_REF.search(moveit_model_path.read_text(encoding="utf-8"))
    return match.group(1).strip() if match else None


def read_moveit_model_planning_group(moveit_model_path: Path) -> str:
    if not moveit_model_path.is_file():
        return ""
    match = _PLANNING_GROUP.search(moveit_model_path.read_text(encoding="utf-8"))
    return match.group(1).strip() if match else ""


def patch_moveit_model_descriptor_ref(text: str, expected: str) -> tuple[str, bool]:
    """把 MODEL_DESCRIPTOR.model_ref 改成 PointSet 使用的领域身份。"""

    expected = str(expected).strip()
    if not expected:
        raise ValueError("expected model_ref 不能为空")
    match = _MODEL_DESCRIPTOR_REF.search(text)
    if match is None:
        raise ValueError("moveit_model.py 未找到 MODEL_DESCRIPTOR.model_ref")
    current = match.group(1).strip()
    if current == expected:
        return text, False
    updated = text[: match.start(1)] + expected + text[match.end(1) :]
    return updated, True


def read_arm_model_facts(
    *,
    moveit_model_path: Path,
    domain_pkg: str,
    device_id: str,
) -> ArmModelFacts:
    """读取 flange_frame / 关节名；endpoint 与规划组只绑 device_id。"""

    normalized_device = str(device_id).strip()
    if not normalized_device:
        raise ValueError("device_id 不能为空")
    if not moveit_model_path.is_file():
        raise FileNotFoundError(f"缺少 moveit_model.py: {moveit_model_path}")
    text = moveit_model_path.read_text(encoding="utf-8")
    model_yaml_path = moveit_model_path.parent / "models" / "model.yaml"
    yaml_joints, _yaml_moveit_group = _read_model_yaml(model_yaml_path)
    py_joints = _parse_canonical_joint_names(text)

    flange_match = _FLANGE.search(text)
    flange_frame = flange_match.group(1).strip() if flange_match else ""
    if not flange_frame:
        raise ValueError("moveit_model.py 未声明 flange_frame")

    canonical_joint_names = yaml_joints or py_joints
    if not canonical_joint_names:
        raise ValueError("无法从 model.yaml 或 moveit_model.py 读取 kinematic_joints")

    distribution = f"{domain_pkg.replace('_', '-')}-{normalized_device.replace('_', '-')}"
    return ArmModelFacts(
        device_id=normalized_device,
        flange_frame=flange_frame,
        distribution=distribution,
        arm_endpoint_suffix=_ARM_ROLE_SUFFIX,
        canonical_joint_names=canonical_joint_names,
        moveit_group=expected_moveit_group(normalized_device),
        arm_endpoint=expected_arm_endpoint(normalized_device),
        model_yaml_path=model_yaml_path if model_yaml_path.is_file() else None,
    )


def read_py_canonical_joint_names(moveit_model_path: Path) -> tuple[str, ...]:
    """读取 moveit_model.py 中 _CANONICAL_JOINT_NAMES。"""

    if not moveit_model_path.is_file():
        return ()
    return _parse_canonical_joint_names(moveit_model_path.read_text(encoding="utf-8"))


def read_moveit_model_joint_mapping(moveit_model_path: Path) -> tuple[str, ...]:
    """读取 moveit_model.py 中 _JOINT_NAMES 映射到的 canonical 关节名。"""

    if not moveit_model_path.is_file():
        return ()
    return _parse_joint_name_values(moveit_model_path.read_text(encoding="utf-8"))
