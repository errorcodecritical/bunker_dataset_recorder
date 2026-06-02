# Copyright (c) Sensrad 2025

"""Launch file for oden_mqtt_bridge"""

# pylint: disable=no-name-in-module
# pylint: disable=import-self

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Standard launch file for OdenMqttBridge"""

    return LaunchDescription(
        [
            DeclareLaunchArgument("log-level", default_value="info"),
            DeclareLaunchArgument("broker_uri", default_value="tcp://127.0.0.1:1883"),
            DeclareLaunchArgument("client_id", default_value="oden_mqtt_bridge"),
            Node(
                package="oden_mqtt_bridge",
                namespace="sensrad/radar_1",
                respawn=True,
                respawn_delay=2,
                executable="oden_mqtt_bridge",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log-level"),
                ],
                parameters=[
                    {
                        "broker_uri": LaunchConfiguration("broker_uri"),
                        "client_id": LaunchConfiguration("client_id"),
                    }
                ],
            ),
        ]
    )
