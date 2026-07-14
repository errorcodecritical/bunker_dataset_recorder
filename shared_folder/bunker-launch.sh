#!/bin/bash
set -e

#ROS 2 Middleware Implementation 

#Fastrtps
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

#Cyclonedds
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

#Build workspace only with the packages descriminated on docker compose file
cd /root/ros2_ws/
colcon build --symlink-install

# Source ROS2 Workspace
source /root/ros2_ws/install/setup.bash

ip link set can2 up type can bitrate 500000

#Run Bunker pkg
ros2 launch bunker_base bunker_base.launch.py use_sim_time:=false port_name:=can2 odom_frame:=odom base_frame:=base_link odom_topic_name:=bunker_odom is_bunker_mini:=true simulated_robot:=false control_rate:=50 &

#Run bunker URDF publisher
ros2 launch bunker_description robot_state_publisher.launch.py &

ros2 launch twist_mux twist_mux.launch.py config_locks:=/config/bunker/twist_mux.yaml config_topics:=/config/bunker/twist_mux.yaml &
ros2 launch teleop_twist_joy teleop-launch.py config_filepath:=/config/bunker/teleop_twist_joy.yaml