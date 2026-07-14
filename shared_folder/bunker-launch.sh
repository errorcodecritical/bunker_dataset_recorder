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
cd /root/ros2_ws/
colcon build --symlink-install

# Source ROS2 Workspace
source /root/ros2_ws/install/setup.bash

#Run realsense driver
ros2 launch bunker_description robot_state_publisher.launch.py & \
ros2 launch twist_mux twist_mux.launch.py config_locks:=/config/bunker/twist_mux.yaml config_topics:=/config/bunker/twist_mux.yaml & \
ros2 launch teleop_twist_joy teleop-launch.py config_filepath:=/config/bunker/teleop_twist_joy.yaml