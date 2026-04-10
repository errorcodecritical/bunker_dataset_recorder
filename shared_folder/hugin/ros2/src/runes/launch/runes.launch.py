# Copyright (c) Sensrad 2025
"""
Launch the Runes perception visualizer node.

ros2 launch runes runes.launch.py

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch file for the Runes perception visualizer."""
    # Launch arguments
    log_level_arg = DeclareLaunchArgument(
        "log-level",
        default_value="WARN",
        description="Logging level for the visualizer node",
    )

    param_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([FindPackageShare("runes"), "config", "runes.yaml"]),
        description="YAML file with parameters for Runes",
    )

    # Path to the YAML selected by the user
    params = [LaunchConfiguration("params_file")]

    # Perception visualizer node
    visualizer_node = Node(
        package="runes",
        executable="perception_visualizer_node",
        name="runes",
        parameters=params,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log-level")],
        output="screen",
    )

    # Launch
    return LaunchDescription(
        [
            log_level_arg,
            param_file_arg,
            visualizer_node,
        ]
    )
