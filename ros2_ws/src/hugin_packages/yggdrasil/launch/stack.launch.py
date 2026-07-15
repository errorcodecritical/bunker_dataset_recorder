# Copyright (c) Sensrad 2025
"""Launch file for stacking multiple radars and visualiser.

Launches:
    • Multiple Sensrad groups (each Sensrad group contains:
        one Hugin D1,
        one Oden perception,
        one Runes visualiser)
    • Ymir node for aggregated odometry (handles single or multiple radars)
    • Static TFs for Sensrad stack, following the REP-105 frame conventions
    • Visualiser: Foxglove Studio or RViz 2
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch file for stacking multiple radars and visualiser."""

    # Launch arguments
    args = [
        DeclareLaunchArgument(
            "root_namespace",
            default_value="sensrad",
            description="Root namespace of radar namespaces",
        ),
        DeclareLaunchArgument(
            "radar_count",
            default_value="1",
            description="How many radar instances to start",
        ),
        DeclareLaunchArgument(
            "extrinsics_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("yggdrasil"), "extrinsics", "extrinsics_hugin.yaml"]
            ),
            description="YAML with static TFs",
        ),
        DeclareLaunchArgument(
            "visualizer",
            default_value="foxglove",
            description="'foxglove', 'rviz2' or 'none'",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("yggdrasil"), "rviz", "default.rviz"]
            ),
            description="Path to RViz configuration file",
        ),
        DeclareLaunchArgument("log_level", default_value="WARN"),
    ]

    def build_launch(context, *_, **__):
        """Build launch function"""
        root_namespace = LaunchConfiguration("root_namespace").perform(context)
        radar_count = int(LaunchConfiguration("radar_count").perform(context))
        extrinsics = LaunchConfiguration("extrinsics_file").perform(context)
        log_level = LaunchConfiguration("log_level").perform(context)
        visualizer = LaunchConfiguration("visualizer").perform(context)
        rviz_config_file = LaunchConfiguration("rviz_config_file")

        yggdrasil_pkg = FindPackageShare("yggdrasil")
        ymir_pkg = FindPackageShare("ymir")

        nodes = []
        #  # Launch static TFs
        # nodes.append(
        #     IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource(
        #             [PathJoinSubstitution([yggdrasil_pkg, "launch", "static_tf.launch.py"])]
        #         ),
        #         launch_arguments={
        #             "extrinsics_file": extrinsics,
        #             "tf_prefix": "",  # customer bundle never prefixes
        #         }.items(),
        #     )
        # )

        # Launch N number of Sensrad groups and build list of radar namespaces
        radar_core_path = PathJoinSubstitution([yggdrasil_pkg, "launch", "sensrad_group.launch.py"])

        for idx in range(1, radar_count + 1):
            # Namespace for this Sensrad instance
            radar_namespace = f"{root_namespace}/radar_{idx}"
            frame_id = f"radar_{idx}"

            # Parameters file for this Sensrad instance
            # Note: Requires specific name of parameters files
            # e.g. sensrad_params_1.yaml, sensrad_params_2.yaml.
            params_file = PathJoinSubstitution(
                [FindPackageShare("yggdrasil"), "config", f"sensrad_params_{idx}.yaml"]
            )

            nodes.append(
                GroupAction(
                    [
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(radar_core_path),
                            launch_arguments={
                                "radar_namespace": radar_namespace,
                                "frame_id": frame_id,
                                "params_file": params_file,
                                "log_level": log_level,
                            }.items(),
                        ),
                    ]
                )
            )

        # Launch Ymir for aggregation of results from multiple radars; currently only
        # odometry results are fused.
        ymir_params_file = PathJoinSubstitution([ymir_pkg, "config", "ymir.yaml"])
        ymir_launch = GroupAction(
            [
                PushRosNamespace(f"{root_namespace}"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([ymir_pkg, "launch", "ymir.launch.py"])
                    ),
                    launch_arguments={
                        "params_file": ymir_params_file,
                        "log-level": log_level,
                    }.items(),
                ),
            ]
        )
        nodes.append(ymir_launch)

        # Visualiser
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("yggdrasil"), "launch", "visual.launch.py"]
                    )
                ),
                launch_arguments={
                    "visualizer": visualizer,
                    "rviz_config_file": rviz_config_file,
                }.items(),
            )
        )

        return nodes

    # Return launch description
    return LaunchDescription(args + [OpaqueFunction(function=build_launch)])
