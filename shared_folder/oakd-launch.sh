#!/bin/bash
set -e

#ROS 2 Middleware Implementation 

#Fastrtps
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

#Cyclonedds
#export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

#Launch oakd v3 driver
#ros2 launch depthai_ros_driver_v3 driver.launch.py

#Launch oakd v2 driver
ros2 launch depthai_ros_driver camera.launch.py