# Copyright (c) Sensrad 2025
"""Launch file for the Oden perception Nodes"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file for the Oden perception Nodes"""

    log_level_arg = DeclareLaunchArgument("log_level", default_value="WARN")

    # point at your package’s config folder, not the launch folder
    params = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [
                TextSubstitution(text=get_package_share_directory("oden")),
                "config",
                "oden.yaml",
            ]
        ),
        description="YAML file containing additional parameters for the driver",
    )
    oden_node = Node(
        package="oden",
        executable="oden",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription(
        [
            log_level_arg,
            params,
            oden_node,
        ]
    )
