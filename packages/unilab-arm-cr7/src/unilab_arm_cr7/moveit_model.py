"""从 CR7 distribution 固有资产构造可命名空间化的六轴 MoveIt 模型。"""

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

from .adapters.moveit import CR7_JOINT_NAMES

_SOURCE_DIGEST = "c64ced0cdcb2654dc07190dc0b9c4d3db813a9515507a69c5c027d82547c14c7"
_MODEL_ROOT = Path(__file__).resolve().parent / "models"
_MODEL_DESCRIPTOR = _MODEL_ROOT / "model.yaml"
_SOURCE_URDF = _MODEL_ROOT / "cr7_robot.urdf"
_MESH_ROOT = _MODEL_ROOT / "meshes" / "cr7"
_LINK_NAMES = {
    "dummy_link": "device_link",
    "base_link": "cr7_base",
    **{f"Link{index}": f"cr7_link_{index}" for index in range(1, 7)},
}
_JOINT_NAMES = {
    "dummy_joint": "cr7_base_mount_joint",
    **{f"joint{index}": f"cr7_joint_{index}" for index in range(1, 7)},
}
_JOINT_EFFORT = (150.0, 150.0, 150.0, 30.0, 30.0, 30.0)
_DISABLED_COLLISIONS = (
    ("cr7_base", "cr7_link_1", "Adjacent"),
    ("cr7_base", "cr7_link_2", "Never"),
    ("cr7_base", "cr7_link_4", "Never"),
    ("cr7_link_1", "cr7_link_2", "Adjacent"),
    ("cr7_link_1", "cr7_link_4", "Never"),
    ("cr7_link_2", "cr7_link_3", "Adjacent"),
    ("cr7_link_3", "cr7_link_4", "Adjacent"),
    ("cr7_link_4", "cr7_link_5", "Adjacent"),
    ("cr7_link_4", "cr7_link_6", "Never"),
    ("cr7_link_5", "cr7_link_6", "Adjacent"),
)
_ARM_SPEC = SixAxisArmModelSpec(
    source_urdf=_SOURCE_URDF,
    expected_source_digest=_SOURCE_DIGEST,
    mesh_paths=tuple(
        _MESH_ROOT / name
        for name in ("base_link0.STL", *(f"J{index}.STL" for index in range(1, 7)))
    ),
    link_names=_LINK_NAMES,
    joint_names=_JOINT_NAMES,
    joint_effort=_JOINT_EFFORT,
    canonical_joint_names=CR7_JOINT_NAMES,
    flange_frame="cr7_tool0",
    last_link="cr7_link_6",
    disabled_collisions=_DISABLED_COLLISIONS,
    mock_system_suffix="cr7",
    model_descriptor_path=_MODEL_DESCRIPTOR,
)


def build_joint_state_name_map(
    *,
    device_id: str,
    source: str = "canonical",
) -> JointStateNameMap:
    """按型号包已验证 source 构造 CR7 exact 反馈映射。"""

    return kit_build_joint_state_name_map(
        device_id=device_id,
        canonical_joint_names=CR7_JOINT_NAMES,
        model_descriptor_path=_MODEL_DESCRIPTOR,
        source=source,
    )


@lru_cache(maxsize=1)
def _load_joint_state_source_mappings() -> dict[str, dict[str, str]]:
    """保留旧测试入口：从型号描述符加载 joint-state 映射。"""

    from unilab_robot_model_kit import load_joint_state_source_mappings

    return load_joint_state_source_mappings(
        str(_MODEL_DESCRIPTOR.resolve()),
        CR7_JOINT_NAMES,
    )


def build_moveit_model(
    *,
    device_id: str,
    position: Mapping[str, Any] | None = None,
    rotation: Mapping[str, Any] | None = None,
    mount_yaw_deg: float = 0.0,
) -> MoveItModelBundle:
    """构造带 Device 命名空间和世界安装位姿的 CR7 MoveIt 模型。"""

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
