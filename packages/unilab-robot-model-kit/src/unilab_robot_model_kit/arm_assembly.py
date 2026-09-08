"""Six-axis MoveIt model assembly pipeline."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .controllers import (
    default_joint_limits,
    default_kinematics,
    moveit_controllers,
    ros2_controllers,
)
from .descriptors import build_joint_state_name_map, validate_device_id
from .digest import compute_topology_digest, sha256_bytes, sha256_file
from .srdf_arm import build_srdf_arm_chain
from .types import MoveItModelBundle
from .urdf_arm import (
    append_flange_frame,
    append_mock_ros2_control,
    insert_fixed_mount_yaw,
    normalize_mount_yaw_deg,
    qualify_robot_tree,
    rewrite_render_mesh_uris,
    world_mount_joint,
)


@dataclass(frozen=True, slots=True)
class SixAxisArmModelSpec:
    """Catalog or domain-owned six-axis arm model facts."""

    model_slug: str
    source_urdf: Path
    expected_source_digest: str
    mesh_paths: tuple[Path, ...]
    link_names: Mapping[str, str]
    joint_names: Mapping[str, str]
    joint_effort: tuple[float, ...]
    canonical_joint_names: tuple[str, ...]
    flange_frame: str
    last_link: str
    disabled_collisions: Sequence[tuple[str, str, str]]
    mock_system_suffix: str
    model_descriptor_path: Path
    srdf_base_link: str = "device_link"


def assemble_six_axis_moveit_model(
    spec: SixAxisArmModelSpec,
    *,
    device_id: str,
    position: Mapping[str, Any] | None = None,
    rotation: Mapping[str, Any] | None = None,
    mount_yaw_deg: float = 0.0,
    initial_joint_values: tuple[float, ...] | None = None,
) -> MoveItModelBundle:
    """Build namespaced six-axis MoveIt bundle from locked vendor URDF."""

    normalized_device_id = validate_device_id(device_id)
    source_bytes = spec.source_urdf.read_bytes()
    source_digest = sha256_bytes(source_bytes)
    if source_digest != spec.expected_source_digest:
        raise ValueError(f"{spec.model_slug.upper()} 固有 URDF 摘要漂移")
    missing_meshes = tuple(
        path.name for path in spec.mesh_paths if not path.is_file()
    )
    if missing_meshes:
        raise ValueError(f"{spec.model_slug.upper()} mesh 资产缺失: " + ", ".join(missing_meshes))
    normalized_yaw_deg = normalize_mount_yaw_deg(mount_yaw_deg)

    prefix = f"{normalized_device_id}_"
    render_root = ET.fromstring(source_bytes)
    render_root.set("name", f"{normalized_device_id}_{spec.model_slug}")
    qualify_robot_tree(
        render_root,
        prefix=prefix,
        link_names=spec.link_names,
        joint_names=spec.joint_names,
        mesh_paths=spec.mesh_paths,
        joint_effort=spec.joint_effort,
        model_label=spec.model_slug.upper(),
    )
    append_flange_frame(
        render_root,
        prefix=prefix,
        flange_frame=spec.flange_frame,
        parent_link=spec.last_link,
    )
    execution_root = deepcopy(render_root)
    rewrite_render_mesh_uris(render_root, device_id=normalized_device_id)
    insert_fixed_mount_yaw(
        execution_root,
        prefix=prefix,
        mount_yaw_deg=normalized_yaw_deg,
    )
    insert_fixed_mount_yaw(
        render_root,
        prefix=prefix,
        mount_yaw_deg=normalized_yaw_deg,
    )
    execution_root.insert(0, world_mount_joint(prefix, position, rotation))
    append_mock_ros2_control(
        execution_root,
        prefix=prefix,
        canonical_joint_names=spec.canonical_joint_names,
        mock_system_suffix=spec.mock_system_suffix,
        initial_joint_values=initial_joint_values,
    )

    name_map = build_joint_state_name_map(
        device_id=normalized_device_id,
        canonical_joint_names=spec.canonical_joint_names,
        model_descriptor_path=spec.model_descriptor_path,
    )
    qualified_joints = name_map.qualified_joint_names
    topology_digest = compute_topology_digest(
        payload={
            "device_id": normalized_device_id,
            "model": spec.model_slug,
            "source_digest": source_digest,
            "joint_names": qualified_joints,
            "flange_frame": spec.flange_frame,
        }
    )
    planning_group = f"{prefix}{spec.model_slug}_arm"
    controller_name = f"{prefix}{spec.model_slug}_controller"
    return MoveItModelBundle(
        execution_urdf=ET.tostring(execution_root, encoding="unicode"),
        render_urdf=ET.tostring(render_root, encoding="unicode"),
        srdf=build_srdf_arm_chain(
            robot_name=f"{prefix}{spec.model_slug}",
            prefix=prefix,
            planning_group=planning_group,
            base_link=spec.srdf_base_link,
            tip_link=spec.flange_frame,
            disabled_collisions=spec.disabled_collisions,
        ),
        ros2_controllers=ros2_controllers(
            controller_name=controller_name,
            joint_names=qualified_joints,
        ),
        moveit_controllers=moveit_controllers(
            controller_name=controller_name,
            joint_names=qualified_joints,
        ),
        kinematics=default_kinematics(planning_group=planning_group),
        joint_limits=default_joint_limits(joint_names=qualified_joints),
        source_digest=source_digest,
        mesh_paths=spec.mesh_paths,
        qualified_joint_names=qualified_joints,
        topology_digest=topology_digest,
    )


def verify_locked_source_digest(path: Path, expected_digest: str) -> str:
    """Verify URDF or descriptor digest and return actual digest."""

    actual = sha256_file(path)
    if actual != expected_digest:
        raise ValueError(f"锁定摘要漂移: {path.name}")
    return actual
