# Copyright (c) Sensrad 2025
"""Launch file for the Sensrad platform."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch file for the Sensrad platform."""

    # Launch arguments
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="WARN",
        description="The log level for the Sensrad nodes.",
    )
    visualizer_arg = DeclareLaunchArgument(
        "visualizer",
        default_value="rviz2",
        description="Which visualiser to launch: 'foxglove', 'rviz2', or 'none'.",
    )
    extrinsics_file_arg = DeclareLaunchArgument(
        "extrinsics_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("yggdrasil"), "extrinsics", "extrinsics_hugin.yaml"]
        ),
        description="Path to the extrinsics file.",
    )
    radar_params_arg = DeclareLaunchArgument(
        "radar_params",
        default_value=PathJoinSubstitution(
            [FindPackageShare("yggdrasil"), "config", "sensrad_params_1.yaml"]
        ),
        description="Path to the radar parameters file.",
    )

    # Short form for package share directory
    log_level = LaunchConfiguration("log_level")
    visualizer = LaunchConfiguration("visualizer")
    extrinsics_file = LaunchConfiguration("extrinsics_file")
    radar_params = LaunchConfiguration("radar_params")

    # Call the main yggdrasil launch file with the specified arguments
    launch_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("yggdrasil"), "launch", "stack.launch.py"])
        ),
        launch_arguments={
            "extrinsics_file": extrinsics_file,
            "radar_params": radar_params,
            "visualizer": visualizer,
            "log_level": log_level,
        }.items(),
    )

    return LaunchDescription(
        [
            log_level_arg,
            visualizer_arg,
            extrinsics_file_arg,
            radar_params_arg,
            launch_stack,
        ]
    )
