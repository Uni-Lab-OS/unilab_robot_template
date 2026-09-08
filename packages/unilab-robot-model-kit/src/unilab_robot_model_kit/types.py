"""Shared model bundle types for MoveIt and rail kinematics."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True, slots=True)
class MoveItModelBundle:
    """Execution and render URDF share naming while isolating world mounts."""

    execution_urdf: str
    render_urdf: str
    srdf: str
    ros2_controllers: dict[str, Any]
    moveit_controllers: dict[str, Any]
    kinematics: dict[str, Any]
    joint_limits: dict[str, Any]
    source_digest: str
    mesh_paths: tuple[Path, ...]
    qualified_joint_names: tuple[str, ...]
    topology_digest: str
    rviz_required: bool = False

    @property
    def urdf(self) -> str:
        """Alias retained for existing MoveIt launch callers."""

        return self.execution_urdf


@dataclass(frozen=True, slots=True)
class RailKinematicModelBundle:
    """Single-axis prismatic render model for OS projection and FE."""

    render_urdf: str
    source_digest: str
    qualified_joint_names: tuple[str, ...]
    topology_digest: str
    mesh_paths: tuple[Path, ...] = ()
    mount_link: str = ""
