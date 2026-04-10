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

# Install Atlas Eava ROS2 API
cd /root/shared_folder/atlas/
dpkg -i Atlas_AevaCLI_4_0_0_GA_aarch64.deb
dpkg --force-architecture -i Atlas_AevaAPI_4_0_0_GA_aarch64.deb

#Build workspace
cd /root/ros2_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS2 Workspace
source /root/ros2_ws/install/setup.bash

# Run Foxglove server
#ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=9092

# Run Rviz with sensors layout
rviz2 -d /root/shared_folder/isr.rviz