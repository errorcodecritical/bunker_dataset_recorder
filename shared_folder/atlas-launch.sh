#!/bin/bash
set -e

#ROS 2 Middleware Implementation 

#Fastrtps
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

#Cyclonedds
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cd /root/shared_folder/atlas/
dpkg -i Atlas_AevaCLI_4_0_0_GA_aarch64.deb
dpkg --force-architecture -i Atlas_AevaAPI_4_0_0_GA_aarch64.deb

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

#Build workspace
cd /root/ros2_ws/
colcon build --symlink-install

# Source ROS2 Workspace
source /root/ros2_ws/install/setup.bash

# Run Atlas ROS2
ros2 run ros2_aeva_publisher ros2_aeva_publisher 10.1.14.24 ATLAS