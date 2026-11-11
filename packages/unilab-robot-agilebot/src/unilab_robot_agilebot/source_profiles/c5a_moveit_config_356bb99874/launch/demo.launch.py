"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:

This file is used to start MoveIt and RViz for robotic arm motion planning and visualization.
functions:
- Start the MoveIt node to support motion planning.
- Start RViz for visualization.
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "c5a_description", package_name="c5a_moveit_config"
    ).to_moveit_configs()
    return generate_demo_launch(moveit_config)
