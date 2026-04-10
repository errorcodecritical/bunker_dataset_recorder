# Copyright (c) Sensrad 2025
"""Launch file for one Sensrad group.
Launches:
    • One Hugin D1 radar
    • One Hugin D1 GUI
    • One Oden perception node
    • One Runes visualiser
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch file for one Sensrad group."""

    # Launch arguments
    radar_ns_arg = DeclareLaunchArgument(
        "radar_namespace",
        default_value="sensrad/radar_1",
        description="Namespace to push all radar related nodes into.",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="WARN",
        description="RCUTILS log level (debug|info|warn|error|fatal)",
    )
    # Frame ID
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="radar_1",
        description="Frame ID for the radar data.",
    )

    radar_ns = LaunchConfiguration("radar_namespace")
    log_level = LaunchConfiguration("log_level")
    frame_id = LaunchConfiguration("frame_id")

    # Parameter file
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("yggdrasil"), "config", "sensrad_params_1.yaml"]
        ),
        description="Path to the parameters file.",
    )
    params_file = LaunchConfiguration("params_file")

    hugin_d1_pkg = FindPackageShare("hugin_d1")
    hugin_d1_gui_pkg = FindPackageShare("hugin_d1_gui")
    oden_pkg = FindPackageShare("oden")
    runes_pkg = FindPackageShare("runes")

    hugin_d1_group = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([hugin_d1_pkg, "launch", "hugin_d1.launch.py"])
                ),
                launch_arguments={
                    "params_file": params_file,
                    "log-level": log_level,
                    "frame_id": frame_id,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([hugin_d1_gui_pkg, "launch", "hugin_d1_gui.launch.py"])
                ),
                launch_arguments={"log_level": log_level}.items(),
            ),
        ]
    )

    # Oden perception group with runes
    oden_group = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([oden_pkg, "launch", "oden.launch.py"])
                ),
                launch_arguments={
                    "params_file": params_file,
                    "log_level": log_level,
                }.items(),
            ),
            GroupAction(
                [
                    PushRosNamespace("oden"),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([runes_pkg, "launch", "runes.launch.py"])
                        ),
                        launch_arguments={
                            "params_file": params_file,
                            "log-level": log_level,
                        }.items(),
                    ),
                ]
            ),
        ]
    )

    # Group all Sensrad related nodes under the specified namespace
    sensrad_group = GroupAction(
        actions=[
            PushRosNamespace(radar_ns),
            hugin_d1_group,
            oden_group,
        ]
    )

    return LaunchDescription(
        [
            # Launch arguments
            radar_ns_arg,
            log_level_arg,
            frame_id_arg,
            params_file_arg,
            # Node launches
            sensrad_group,
        ]
    )
