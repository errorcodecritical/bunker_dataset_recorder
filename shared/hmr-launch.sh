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

#Build workspace
cd /ros2_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS2 Workspace
source /ros2_ws/install/setup.bash

ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id odom --child-frame-id base_link &

ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:=/ros2_ws/src/hmr_localisation/config/gt_ouster_ndt_tree.yaml \
  cloud_topic:=/hesai/points imu_topic:=/imu/data use_sim_time:=true \
  global_frame_id:=map odom_frame_id:=odom base_frame_id:=base_link \
  use_imu_preintegration:=true imu_preintegration_use_base_frame_transform:=true \
  publish_lidar_tf:=false lidar_frame_id:=hesai_lidar \
  lidar_tf_x:=0.1105 lidar_tf_y:=0.0 lidar_tf_z:=0.404 lidar_tf_yaw:=3.14159265 \
  publish_imu_tf:=false imu_frame_id:=imu \
  imu_tf_x:=0.062 imu_tf_y:=0.0 imu_tf_z:=0.015 imu_tf_yaw:=1.570796
