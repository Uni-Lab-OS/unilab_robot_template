"""Shared MoveIt and rail model assembly utilities."""

from __future__ import annotations

from .arm_assembly import (
    SixAxisArmModelSpec,
    assemble_six_axis_moveit_model,
    verify_locked_source_digest,
)
from .controllers import (
    default_joint_limits,
    default_kinematics,
    moveit_controllers,
    ros2_controllers,
)
from .descriptors import (
    build_joint_state_name_map,
    is_allowed_arm_provider,
    is_catalog_arm_provider,
    is_domain_arm_provider,
    load_joint_state_source_mappings,
    load_model_descriptor,
    validate_device_id,
)
from .digest import compute_topology_digest, sha256_bytes, sha256_file, sha256_normalized_yaml
from .rail_assembly import PrismaticRailModelSpec, build_prismatic_rail_kinematic_model
from .srdf_arm import build_srdf_arm_chain
from .types import MoveItModelBundle, RailKinematicModelBundle
from .urdf_arm import (
    append_flange_frame,
    append_mock_ros2_control,
    insert_fixed_mount_yaw,
    normalize_mount_yaw_deg,
    qualify_robot_tree,
    rewrite_render_mesh_uris,
    vector_to_xyz,
    world_mount_joint,
)

__all__ = [
    "MoveItModelBundle",
    "PrismaticRailModelSpec",
    "RailKinematicModelBundle",
    "SixAxisArmModelSpec",
    "append_flange_frame",
    "append_mock_ros2_control",
    "assemble_six_axis_moveit_model",
    "build_joint_state_name_map",
    "build_prismatic_rail_kinematic_model",
    "build_srdf_arm_chain",
    "compute_topology_digest",
    "default_joint_limits",
    "default_kinematics",
    "insert_fixed_mount_yaw",
    "is_allowed_arm_provider",
    "is_catalog_arm_provider",
    "is_domain_arm_provider",
    "load_joint_state_source_mappings",
    "load_model_descriptor",
    "moveit_controllers",
    "normalize_mount_yaw_deg",
    "qualify_robot_tree",
    "rewrite_render_mesh_uris",
    "ros2_controllers",
    "sha256_bytes",
    "sha256_file",
    "sha256_normalized_yaml",
    "validate_device_id",
    "verify_locked_source_digest",
    "vector_to_xyz",
    "world_mount_joint",
]
