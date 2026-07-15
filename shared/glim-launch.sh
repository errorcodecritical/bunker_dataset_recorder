#!/bin/bash
set -e

#ROS 2 Middleware Implementation 

#Fastrtps
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

#Cyclonedds
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS2 Humble
source /opt/ros/humble/install/setup.bash

#Build workspace only with the packages descriminated on docker compose file
cd /ros2_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF

#ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id global_map --child-frame-id map &

# Source ROS2 Workspace
source /glim_ws/install/setup.bash

ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/glim_ws/src/glim/config