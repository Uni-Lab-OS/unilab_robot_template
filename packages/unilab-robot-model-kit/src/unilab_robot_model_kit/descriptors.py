"""Model descriptor validation and joint-state mapping loaders."""

from __future__ import annotations

import re
from collections.abc import Mapping
from functools import lru_cache
from pathlib import Path
from typing import Any

import yaml
from unilab_robot_contracts import JointStateNameMap

_DEVICE_ID = re.compile(r"^[A-Za-z0-9_]+$")
_CATALOG_ARM_PROVIDER = re.compile(
    r"^unilab_arm_(?P<slug>[a-z0-9]+):build_moveit_model$"
)
_DOMAIN_ARM_PROVIDER = re.compile(
    r"^(?P<module>[A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z_][A-Za-z0-9_]*)*):build_moveit_model$"
)


def validate_device_id(device_id: str, *, label: str = "device_id") -> str:
    """Normalize and validate Graph device id."""

    normalized = str(device_id).strip()
    if _DEVICE_ID.fullmatch(normalized) is None:
        raise ValueError(f"{label} 只能包含英文、数字和下划线")
    return normalized


def is_catalog_arm_provider(provider: str) -> bool:
    """Return True when provider references template catalog arm package."""

    return _CATALOG_ARM_PROVIDER.fullmatch(str(provider).strip()) is not None


def is_domain_arm_provider(provider: str) -> bool:
    """Return True when provider references a domain-owned arm module."""

    normalized = str(provider).strip()
    if is_catalog_arm_provider(normalized):
        return False
    match = _DOMAIN_ARM_PROVIDER.fullmatch(normalized)
    if match is None:
        return False
    module = str(match.group("module"))
    if module.startswith("unilab_arm_"):
        return False
    return True


def is_allowed_arm_provider(provider: str) -> bool:
    """Accept catalog or domain-owned MoveIt arm providers."""

    return is_catalog_arm_provider(provider) or is_domain_arm_provider(provider)


@lru_cache(maxsize=32)
def load_joint_state_source_mappings(
    model_descriptor_path: str,
    canonical_joint_names: tuple[str, ...],
) -> dict[str, dict[str, str]]:
    """Load joint-state source mappings from ``model.yaml``."""

    path = Path(model_descriptor_path)
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    canonical = tuple(str(value) for value in data.get("kinematic_joints", ()))
    if canonical != canonical_joint_names:
        raise ValueError(f"{path.name} 的 kinematic_joints 与型号实现漂移")
    sources = data.get("joint_state_sources")
    if not isinstance(sources, Mapping) or not sources:
        raise ValueError(f"{path.name} 缺少 joint_state_sources")
    mappings: dict[str, dict[str, str]] = {}
    for source, raw_names in sources.items():
        names = tuple(str(value) for value in raw_names)
        if len(names) != len(canonical) or len(set(names)) != len(names):
            raise ValueError(f"joint-state source 非 exact 六轴: {source}")
        mappings[str(source)] = dict(zip(names, canonical, strict=True))
    return mappings


def build_joint_state_name_map(
    *,
    device_id: str,
    canonical_joint_names: tuple[str, ...],
    model_descriptor_path: Path,
    source: str = "canonical",
) -> JointStateNameMap:
    """Construct exact joint feedback mapping for a six-axis arm."""

    raw_to_canonical = load_joint_state_source_mappings(
        str(model_descriptor_path.resolve()),
        canonical_joint_names,
    ).get(str(source))
    if raw_to_canonical is None:
        raise ValueError(f"型号包未验证 joint-state source: {source}")
    return JointStateNameMap(
        device_id=validate_device_id(device_id),
        canonical_joint_names=canonical_joint_names,
        raw_to_canonical=raw_to_canonical,
    )


def load_model_descriptor(path: Path) -> dict[str, Any]:
    """Parse ``unilab.robot-model/v1`` descriptor YAML."""

    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise TypeError(f"{path} 必须是 YAML 对象")
    return data
