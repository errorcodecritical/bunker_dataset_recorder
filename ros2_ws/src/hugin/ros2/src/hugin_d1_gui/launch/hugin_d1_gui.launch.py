# Copyright (c) Sensrad 2025
#
"""Standard launch file for Hugin D1 GUI"""

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch GUI with in namespace with parameters"""

    log_level_arg = DeclareLaunchArgument("log_level", default_value="WARN")

    gui_node = Node(
        package="hugin_d1_gui",
        executable="hugin_d1_gui",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[{}],
    )

    return LaunchDescription(
        [
            log_level_arg,
            gui_node,
        ]
    )
