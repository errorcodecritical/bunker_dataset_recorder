# Copyright (c) Sensrad 2025
"""Launch file for the Hugin D1 radar driver using composable containers"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file for the Hugin D1 radar driver"""

    log_level_arg = DeclareLaunchArgument("log_level", default_value="WARN")
    frame_id_arg = DeclareLaunchArgument("frame_id", default_value="radar_1")

    # Parameter configuration
    params = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [
                TextSubstitution(text=get_package_share_directory("hugin_d1")),
                "config",
                "hugin_d1.yaml",
            ]
        ),
        description="YAML file containing additional parameters for the driver",
    )
    # Common node arguments
    common_node_kwargs = {
        "package": "hugin_d1",
        "respawn": True,
        "respawn_delay": 1,
        "arguments": ["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    }
    # Radar control node
    control_node = Node(
        executable="control_node",
        name="control_node",
        **common_node_kwargs,
        parameters=[LaunchConfiguration("params_file")],
    )
    # Radar parser node
    parser_node = Node(
        executable="parser_node",
        name="parser_node",
        parameters=[{"frame_id": LaunchConfiguration("frame_id")}],
        **common_node_kwargs,
    )

    return LaunchDescription(
        [
            frame_id_arg,
            log_level_arg,
            params,
            control_node,
            parser_node,
        ]
    )
