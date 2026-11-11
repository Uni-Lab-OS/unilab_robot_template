"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
This file is used to launch MoveIt and RViz for robotic arm simulation and debugging in Gazebo.

functions:
- Start MoveIt nodes to support motion planning.
- Start RViz for visualization.

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    DeclareBooleanLaunchArg,
    add_debuggable_node,
)
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    # Create MoveItConfigsBuilder object to generate MoveIt configuration
    moveit_config = MoveItConfigsBuilder(
        "c5a_description", package_name="c5a_moveit_config"
    ).to_moveit_configs()

    ld = LaunchDescription()

    # move_group
    gbt_generate_move_group_launch(ld, moveit_config)
    # rviz
    gbt_generate_moveit_rviz_launch(ld, moveit_config)

    return ld


def gbt_generate_move_group_launch(ld, moveit_config):
    """
    Configure and add the move_group node's launch arguments and nodes to the LaunchDescription.
    """
    # Add a boolean type of launch parameter for debugging, the default value is False
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    # add a boolean type of launch parameter for allowing trajectory execution, the default value is True
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    # add a boolean type of launch parameter for publishing monitored planning scene, the default value is True
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

    # do not copy dynamics information from /joint_states to internal robot monitoring
    # default to false, because almost nothing in move_group relies on this information
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]
    move_group_params.append({"use_sim_time": True})

    # add a debuggable node for move_group
    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": ":0"},
    )
    return ld


def gbt_generate_moveit_rviz_launch(ld, moveit_config):
    """
    Add the RViz node's launch arguments to the LaunchDescription.
    """

    # add a boolean type of launch parameter for debugging, the default value is False
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    # add a launch argument for RViz configuration file, default value is config/moveit.rviz
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    # define rviz parameters
    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
    ]

    # add a parameter, use simulation time
    rviz_parameters.append({"use_sim_time": True})

    # add a debuggable node for rviz
    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )

    return ld
