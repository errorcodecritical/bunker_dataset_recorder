# Copyright (c) Sensrad 2025
"""
Launch the Ymir multi-radar odometry aggregation node.

ros2 launch ymir ymir.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch file for the Ymir multi-radar odometry aggregator."""
    # Launch arguments
    log_level_arg = DeclareLaunchArgument(
        "log-level",
        default_value="WARN",
        description="Logging level for the Ymir node",
    )

    param_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([FindPackageShare("ymir"), "config", "ymir.yaml"]),
        description="YAML file with parameters for Ymir",
    )

    ymir_node = Node(
        package="ymir",
        executable="ymir_node",
        name="ymir",
        parameters=[LaunchConfiguration("params_file")],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log-level")],
        output="screen",
    )

    # Launch
    return LaunchDescription(
        [
            log_level_arg,
            param_file_arg,
            ymir_node,
        ]
    )
