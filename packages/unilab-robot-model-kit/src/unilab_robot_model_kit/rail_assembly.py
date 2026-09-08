"""Single-axis prismatic rail kinematic model assembly."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .descriptors import validate_device_id
from .digest import compute_topology_digest, sha256_normalized_yaml
from .types import RailKinematicModelBundle


@dataclass(frozen=True, slots=True)
class PrismaticRailModelSpec:
    """Catalog or domain-owned linear rail kinematic facts."""

    model_id: str
    model_descriptor_path: Path
    expected_descriptor_digest: str
    base_link: str
    carriage_link: str
    axis_joint: str
    travel_m: tuple[float, float]
    velocity_limit_m_s: float


def build_prismatic_rail_kinematic_model(
    spec: PrismaticRailModelSpec,
    *,
    device_id: str,
    position: Mapping[str, Any] | None = None,
    rotation: Mapping[str, Any] | None = None,
    qualified_joint_name: str | None = None,
) -> RailKinematicModelBundle:
    """Build namespaced single-axis prismatic rail render URDF."""

    del position, rotation
    normalized = validate_device_id(device_id)
    actual_digest = sha256_normalized_yaml(spec.model_descriptor_path)
    if actual_digest != spec.expected_descriptor_digest:
        raise ValueError("导轨 model.yaml 源摘要漂移")

    prefix = f"{normalized}_"
    base = f"{prefix}{spec.base_link}"
    carriage = f"{prefix}{spec.carriage_link}"
    joint_name = qualified_joint_name or f"{prefix}{spec.axis_joint}"
    lower, upper = spec.travel_m
    velocity = spec.velocity_limit_m_s
    robot = ET.Element("robot", {"name": f"{normalized}_rail"})
    ET.SubElement(robot, "link", {"name": base})
    ET.SubElement(robot, "link", {"name": carriage})
    joint = ET.SubElement(robot, "joint", {"name": joint_name, "type": "prismatic"})
    ET.SubElement(joint, "parent", {"link": base})
    ET.SubElement(joint, "child", {"link": carriage})
    ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(joint, "axis", {"xyz": "1 0 0"})
    ET.SubElement(
        joint,
        "limit",
        {
            "lower": f"{lower}",
            "upper": f"{upper}",
            "effort": "0",
            "velocity": f"{velocity}",
        },
    )
    qualified = (joint_name,)
    return RailKinematicModelBundle(
        render_urdf=ET.tostring(robot, encoding="unicode"),
        source_digest=spec.expected_descriptor_digest,
        qualified_joint_names=qualified,
        topology_digest=compute_topology_digest(
            payload={
                "device_id": normalized,
                "model": spec.model_id,
                "source_digest": spec.expected_descriptor_digest,
                "joint_names": qualified,
            }
        ),
        mount_link=carriage,
    )
