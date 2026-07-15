# Copyright (c) Sensrad 2025
"""Launch file for ROS2 visualizer."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEFAULT_VISUALIZER = "foxglove"  # Options: 'foxglove', 'rviz2', or 'none'


def generate_launch_description():
    """Launch file for ROS2 visualizer."""

    # Launch arguments
    visualizer_arg = DeclareLaunchArgument(
        "visualizer",
        default_value=DEFAULT_VISUALIZER,
        description="Which visualiser to launch: 'foxglove', 'rviz2', or 'none'.",
    )
    rviz_config_file_arg = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=PathJoinSubstitution([FindPackageShare("yggdrasil"), "rviz", "default.rviz"]),
        description="Path to RViz configuration file.",
    )

    # Short form for package share directory
    visualizer = LaunchConfiguration("visualizer")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    # Foxglove Studio and Bridge
    foxglove_studio = ExecuteProcess(
        cmd=["foxglove-studio"],
        condition=IfCondition(PythonExpression(["'", visualizer, "' == 'foxglove'"])),
    )
    foxglove_bridge = ExecuteProcess(
        cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"],
        condition=IfCondition(PythonExpression(["'", visualizer, "' == 'foxglove'"])),
    )

    # RViz2 Node
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        condition=IfCondition(PythonExpression(["'", visualizer, "' == 'rviz2'"])),
    )

    # Return launch description
    return LaunchDescription(
        [
            visualizer_arg,
            rviz_config_file_arg,
            foxglove_bridge,
            foxglove_studio,
            rviz2_node,
        ]
    )
