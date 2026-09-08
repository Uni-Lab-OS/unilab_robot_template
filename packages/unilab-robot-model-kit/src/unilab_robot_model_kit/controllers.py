"""MoveIt and ros2_control parameter builders."""

from __future__ import annotations

from typing import Any


def ros2_controllers(
    *, controller_name: str, joint_names: tuple[str, ...]
) -> dict[str, Any]:
    """Build controller_manager parameters for mock hardware."""

    return {
        "controller_manager": {
            "ros__parameters": {
                controller_name: {
                    "type": "joint_trajectory_controller/JointTrajectoryController"
                }
            }
        },
        controller_name: {
            "ros__parameters": {
                "joints": list(joint_names),
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
                "open_loop_control": True,
            }
        },
    }


def moveit_controllers(
    *, controller_name: str, joint_names: tuple[str, ...]
) -> dict[str, Any]:
    """Build MoveIt simple controller manager parameters."""

    return {
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
        "moveit_simple_controller_manager": {
            "controller_names": [controller_name],
            controller_name: {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": list(joint_names),
            },
        },
    }


def default_kinematics(*, planning_group: str) -> dict[str, Any]:
    """Default KDL kinematics plugin block."""

    return {
        planning_group: {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.05,
        }
    }


def default_joint_limits(*, joint_names: tuple[str, ...]) -> dict[str, Any]:
    """Default velocity-only joint limits block."""

    return {
        "joint_limits": {
            name: {
                "has_velocity_limits": True,
                "max_velocity": 3.14,
                "has_acceleration_limits": False,
                "max_acceleration": 0.0,
            }
            for name in joint_names
        }
    }
