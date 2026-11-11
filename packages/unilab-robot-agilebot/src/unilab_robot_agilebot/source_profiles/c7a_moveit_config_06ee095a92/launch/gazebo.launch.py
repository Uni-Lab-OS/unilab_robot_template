"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
This file is used to start the Gazebo simulation environment and load the robot model.

"""

import os
import re

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def remove_comments(text):
    """
    remove comments from text
    Args:
        text (_type_): _description_

    Returns:
        _type_: _description_
    """
    pattern = r"<!--(.*?)-->"
    return re.sub(pattern, "", text, flags=re.DOTALL)


def generate_launch_description():
    """
    generate launch description
    """

    # define robot type, package name and launch file name
    package_name = "gbt_gazebo"
    launch_name = "gazebo_demo.launch.py"

    # get package path
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    launch_script_path = os.path.join(pkg_share, "launch", launch_name)

    ld = LaunchDescription()

    # launch another launch file using IncludeLaunchDescription
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_script_path),
            # robot_type is used to load the robot model
            launch_arguments=[
                ("robot_type", "C7A"),
                (
                    "controller_name",
                    "gbt_c7a_arm_controller",
                ),  # define in config/moveit_controllers.yaml
            ],
        )
    )

    return ld
