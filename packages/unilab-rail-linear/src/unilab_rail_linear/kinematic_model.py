"""单轴导轨与机械臂共用的关节命名、拓扑摘要和渲染 URDF。"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

from unilab_robot_model_kit import (
    PrismaticRailModelSpec,
    RailKinematicModelBundle,
    build_prismatic_rail_kinematic_model,
)

from .factory import MODEL_DESCRIPTOR, build_joint_state_name_map

_MODEL_YAML = Path(__file__).resolve().parent / "models" / "model.yaml"
_SOURCE_DIGEST = "9ec7d9833f46c26e02e08f06aecd12495e4ab6753ebd1e47a967f7bf885bf83d"
_RAIL_SPEC = PrismaticRailModelSpec(
    model_id="szlab-linear-rail-a",
    model_descriptor_path=_MODEL_YAML,
    expected_descriptor_digest=_SOURCE_DIGEST,
    base_link=MODEL_DESCRIPTOR.base_link,
    carriage_link=MODEL_DESCRIPTOR.carriage_link,
    axis_joint=MODEL_DESCRIPTOR.axis_joint,
    travel_m=MODEL_DESCRIPTOR.travel_m,
    velocity_limit_m_s=MODEL_DESCRIPTOR.velocity_limit_m_s,
)


def build_kinematic_model(
    *,
    device_id: str,
    position: Mapping[str, Any] | None = None,
    rotation: Mapping[str, Any] | None = None,
) -> RailKinematicModelBundle:
    """构造 Device 命名空间下的单轴棱柱关节渲染 URDF。"""

    name_map = build_joint_state_name_map(device_id=str(device_id).strip())
    return build_prismatic_rail_kinematic_model(
        _RAIL_SPEC,
        device_id=device_id,
        position=position,
        rotation=rotation,
        qualified_joint_name=name_map.qualified_joint_names[0],
    )


__all__ = ["RailKinematicModelBundle", "build_kinematic_model"]
