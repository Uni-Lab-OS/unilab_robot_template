"""URDF mutation helpers for six-axis MoveIt arm models."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from pathlib import Path
from typing import Any


def qualify_robot_tree(
    root: ET.Element,
    *,
    prefix: str,
    link_names: Mapping[str, str],
    joint_names: Mapping[str, str],
    mesh_paths: tuple[Path, ...],
    joint_effort: tuple[float, ...],
    model_label: str,
) -> None:
    """Rename vendor URDF links/joints and bind mesh URIs to locked assets."""

    mesh_by_name = {path.name: path for path in mesh_paths}
    for link in root.findall("link"):
        link.set("name", f"{prefix}{link_names[link.attrib['name']]}")
    for joint in root.findall("joint"):
        original_joint = joint.attrib["name"]
        joint.set("name", f"{prefix}{joint_names[original_joint]}")
        for relation in ("parent", "child"):
            element = joint.find(relation)
            if element is None:
                raise ValueError(f"{model_label} URDF joint 缺少 {relation}")
            element.set(
                "link",
                f"{prefix}{link_names[element.attrib['link']]}",
            )
        if original_joint.startswith("joint"):
            index = int(original_joint.removeprefix("joint")) - 1
            limit = joint.find("limit")
            if limit is None:
                raise ValueError(f"{model_label} URDF 旋转关节缺少 limit")
            limit.set("effort", str(joint_effort[index]))
            limit.set("velocity", "3.14")
    for mesh in root.findall(".//mesh"):
        name = Path(mesh.attrib["filename"]).name
        path = mesh_by_name.get(name)
        if path is None:
            raise ValueError(f"{model_label} URDF 引用了未锁定 mesh: {name}")
        mesh.set("filename", path.resolve().as_uri())


def rewrite_render_mesh_uris(root: ET.Element, *, device_id: str) -> None:
    """Use instance-relative mesh URLs for render URDF."""

    for mesh in root.findall(".//mesh"):
        name = Path(mesh.attrib["filename"]).name
        mesh.set("filename", f"{device_id}/meshes/{name}")


def append_flange_frame(
    root: ET.Element,
    *,
    prefix: str,
    flange_frame: str,
    parent_link: str,
) -> None:
    """Append fixed flange frame after the last arm link."""

    qualified_flange = f"{prefix}{flange_frame}"
    ET.SubElement(root, "link", {"name": qualified_flange})
    joint = ET.SubElement(
        root,
        "joint",
        {"name": f"{qualified_flange}_joint", "type": "fixed"},
    )
    ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(joint, "parent", {"link": f"{prefix}{parent_link}"})
    ET.SubElement(joint, "child", {"link": qualified_flange})


def world_mount_joint(
    prefix: str,
    position: Mapping[str, Any] | None,
    rotation: Mapping[str, Any] | None,
) -> ET.Element:
    """Create fixed joint from world to mount-yaw link."""

    xyz = vector_to_xyz(position, scale=0.001)
    rpy = vector_to_xyz(rotation, scale=1.0)
    joint = ET.Element(
        "joint",
        {"name": f"{prefix}world_mount_joint", "type": "fixed"},
    )
    ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": rpy})
    ET.SubElement(joint, "parent", {"link": "world"})
    ET.SubElement(joint, "child", {"link": f"{prefix}mount_yaw_link"})
    return joint


def normalize_mount_yaw_deg(value: object) -> float:
    """Normalize finite Z mount yaw in degrees."""

    yaw = float(value)
    if not math.isfinite(yaw):
        raise ValueError("mount_yaw_deg 必须是有限角度（度）")
    return yaw


def insert_fixed_mount_yaw(
    root: ET.Element,
    *,
    prefix: str,
    mount_yaw_deg: float,
) -> None:
    """Insert fixed mount-yaw joint; not a runtime seventh axis."""

    yaw_link = f"{prefix}mount_yaw_link"
    ET.SubElement(root, "link", {"name": yaw_link})
    joint = ET.SubElement(
        root,
        "joint",
        {"name": f"{prefix}mount_yaw_joint", "type": "fixed"},
    )
    ET.SubElement(
        joint,
        "origin",
        {"xyz": "0 0 0", "rpy": f"0 0 {math.radians(mount_yaw_deg)}"},
    )
    ET.SubElement(joint, "parent", {"link": yaw_link})
    ET.SubElement(joint, "child", {"link": f"{prefix}device_link"})


def append_mock_ros2_control(
    root: ET.Element,
    *,
    prefix: str,
    canonical_joint_names: tuple[str, ...],
    mock_system_suffix: str,
    initial_joint_values: tuple[float, ...] | None = None,
) -> None:
    """Append mock_components ros2_control block for simulation."""

    if initial_joint_values is None:
        initial_joint_values = (0.0,) * len(canonical_joint_names)
    if len(initial_joint_values) != len(canonical_joint_names):
        raise ValueError("initial_joint_values 必须与 canonical_joint_names 等长")
    control = ET.SubElement(
        root,
        "ros2_control",
        {"name": f"{prefix}{mock_system_suffix}_mock_system", "type": "system"},
    )
    hardware = ET.SubElement(control, "hardware")
    ET.SubElement(hardware, "plugin").text = "mock_components/GenericSystem"
    for joint_name, initial_value in zip(
        canonical_joint_names,
        initial_joint_values,
        strict=True,
    ):
        joint = ET.SubElement(control, "joint", {"name": f"{prefix}{joint_name}"})
        ET.SubElement(joint, "command_interface", {"name": "position"})
        state = ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(state, "param", {"name": "initial_value"}).text = str(
            initial_value
        )
        ET.SubElement(joint, "state_interface", {"name": "velocity"})


def vector_to_xyz(value: Mapping[str, Any] | None, *, scale: float) -> str:
    """Convert optional Graph pose mapping to URDF xyz/rpy string."""

    candidate: object = value or {}
    if isinstance(candidate, Mapping) and isinstance(candidate.get("position"), Mapping):
        candidate = candidate["position"]
    mapping = candidate if isinstance(candidate, Mapping) else {}
    return " ".join(str(float(mapping.get(axis, 0.0)) * scale) for axis in "xyz")
