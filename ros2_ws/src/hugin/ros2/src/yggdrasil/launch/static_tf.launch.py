# Copyright (c) Sensrad 2025
"""Launch file for static TFs."""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch file for static TFs."""

    # Launch arguments
    default_extrinsics = PathJoinSubstitution(
        [FindPackageShare("yggdrasil"), "extrinsics", "extrinsics_hugin.yaml"]
    )

    extrinsics_arg = DeclareLaunchArgument(
        "extrinsics_file",
        default_value=default_extrinsics,
        description="Path to YAML file with <static_transforms> mapping.",
    )
    tf_prefix_arg = DeclareLaunchArgument(
        "tf_prefix", default_value="", description="Optional TF frame prefix"
    )

    def pref(frame: str):
        """Prepend the TF prefix to the frame name."""
        return [LaunchConfiguration("tf_prefix"), frame]

    def make_nodes(context, *_):
        """Create static transform nodes from input YAML file."""
        path = os.path.expanduser(LaunchConfiguration("extrinsics_file").perform(context))
        if not os.path.isfile(path):
            raise RuntimeError(f"Extrinsics file not found: {path}")

        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        try:
            transforms = data["static_transforms"]
        except (TypeError, KeyError) as exc:
            raise RuntimeError(
                f"{path} must contain a top-level 'static_transforms:' block"
            ) from exc

        actions = []
        for child, tf in transforms.items():
            xyz = [str(v) for v in tf["xyz"]]
            quat = [str(q) for q in tf["quat"]]
            actions.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"static_tf_{child}",
                    arguments=[
                        "--x",
                        xyz[0],
                        "--y",
                        xyz[1],
                        "--z",
                        xyz[2],
                        "--qx",
                        quat[0],
                        "--qy",
                        quat[1],
                        "--qz",
                        quat[2],
                        "--qw",
                        quat[3],
                        "--frame-id",
                        tf["parent"],
                        "--child-frame-id",
                        pref(child),
                    ],
                    output="screen",
                )
            )
        return actions

    #  Return launch description
    return LaunchDescription(
        [
            extrinsics_arg,
            tf_prefix_arg,
            OpaqueFunction(function=make_nodes),
        ]
    )
