"""从 CR5 distribution 固有资产构造可命名空间化的六轴 MoveIt 模型。"""

from __future__ import annotations

from collections.abc import Mapping
from functools import lru_cache
from pathlib import Path
from typing import Any

from unilab_robot_contracts import JointStateNameMap
from unilab_robot_model_kit import (
    MoveItModelBundle,
    SixAxisArmModelSpec,
    assemble_six_axis_moveit_model,
    build_joint_state_name_map as kit_build_joint_state_name_map,
)

from .adapters.moveit import CR5_JOINT_NAMES

_SOURCE_DIGEST = "8c8b9ea935fd83122b19b572c84d107e81b4864d4310c94d0906cc361e7631c2"
_MODEL_ROOT = Path(__file__).resolve().parent / "models"
_MODEL_DESCRIPTOR = _MODEL_ROOT / "model.yaml"
_SOURCE_URDF = _MODEL_ROOT / "cr5_robot.urdf"
_MESH_ROOT = _MODEL_ROOT / "meshes" / "cr5"
_LINK_NAMES = {
    "dummy_link": "device_link",
    "base_link": "cr5_base",
    **{f"Link{index}": f"cr5_link_{index}" for index in range(1, 7)},
}
_JOINT_NAMES = {
    "dummy_joint": "cr5_base_mount_joint",
    **{f"joint{index}": f"cr5_joint_{index}" for index in range(1, 7)},
}
_JOINT_EFFORT = (150.0, 150.0, 150.0, 30.0, 30.0, 30.0)
_DISABLED_COLLISIONS = (
    ("cr5_base", "cr5_link_1", "Adjacent"),
    ("cr5_base", "cr5_link_2", "Never"),
    ("cr5_base", "cr5_link_4", "Never"),
    ("cr5_link_1", "cr5_link_2", "Adjacent"),
    ("cr5_link_1", "cr5_link_4", "Never"),
    ("cr5_link_2", "cr5_link_3", "Adjacent"),
    ("cr5_link_3", "cr5_link_4", "Adjacent"),
    ("cr5_link_4", "cr5_link_5", "Adjacent"),
    ("cr5_link_4", "cr5_link_6", "Never"),
    ("cr5_link_5", "cr5_link_6", "Adjacent"),
)
_ARM_SPEC = SixAxisArmModelSpec(
    model_slug="cr5",
    source_urdf=_SOURCE_URDF,
    expected_source_digest=_SOURCE_DIGEST,
    mesh_paths=tuple(
        _MESH_ROOT / name
        for name in ("base_link.STL", *(f"J{index}.STL" for index in range(1, 7)))
    ),
    link_names=_LINK_NAMES,
    joint_names=_JOINT_NAMES,
    joint_effort=_JOINT_EFFORT,
    canonical_joint_names=CR5_JOINT_NAMES,
    flange_frame="cr5_tool0",
    last_link="cr5_link_6",
    disabled_collisions=_DISABLED_COLLISIONS,
    mock_system_suffix="cr5",
    model_descriptor_path=_MODEL_DESCRIPTOR,
)


def build_joint_state_name_map(
    *,
    device_id: str,
    source: str = "canonical",
) -> JointStateNameMap:
    """按型号包已验证 source 构造 CR5 exact 反馈映射。"""

    return kit_build_joint_state_name_map(
        device_id=device_id,
        canonical_joint_names=CR5_JOINT_NAMES,
        model_descriptor_path=_MODEL_DESCRIPTOR,
        source=source,
    )


@lru_cache(maxsize=1)
def _load_joint_state_source_mappings() -> dict[str, dict[str, str]]:
    """保留旧测试入口：从型号描述符加载 joint-state 映射。"""

    from unilab_robot_model_kit import load_joint_state_source_mappings

    return load_joint_state_source_mappings(
        str(_MODEL_DESCRIPTOR.resolve()),
        CR5_JOINT_NAMES,
    )


def build_moveit_model(
    *,
    device_id: str,
    position: Mapping[str, Any] | None = None,
    rotation: Mapping[str, Any] | None = None,
    mount_yaw_deg: float = 0.0,
) -> MoveItModelBundle:
    """构造带 Device 命名空间和世界安装位姿的 CR5 MoveIt 模型。"""

    return assemble_six_axis_moveit_model(
        _ARM_SPEC,
        device_id=device_id,
        position=position,
        rotation=rotation,
        mount_yaw_deg=mount_yaw_deg,
    )


__all__ = [
    "MoveItModelBundle",
    "build_joint_state_name_map",
    "build_moveit_model",
]
