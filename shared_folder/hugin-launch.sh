#!/bin/bash
set -e

#ROS 2 Middleware Implementation 

#Fastrtps
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

#Cyclonedds
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

#Build workspace only with the packages descriminated on docker compose file
cd /opt/ros/overlay_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS2 Workspace
source /opt/ros/overlay_ws/install/setup.bash

#Run Hugin driver
ros2 launch yggdrasil sensrad.launch.py visualizer:=none