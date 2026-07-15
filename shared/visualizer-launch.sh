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
cd /root/shared/atlas/
dpkg -i Atlas_AevaCLI_4_0_0_GA_aarch64.deb
dpkg --force-architecture -i Atlas_AevaAPI_4_0_0_GA_aarch64.deb

#Build workspace
cd /root/ros2_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS2 Workspace
source /root/ros2_ws/install/setup.bash

# Run Foxglove server
#ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=9092

#ros2 bag play /root/rosbags/2026_07_03_18_28_32__lab_test_for_glim_ --clock &
#ros2 bag play /root/rosbags/2026_06_29_16_21_05__lab_test_go1_2_ --clock &
#ros2 bag play /root/rosbags/2026_07_06_17_26_27__bunker-kalhan-full_ --clock &

# Run Rviz with sensors layout
rviz2 -d /root/shared/isr.rviz

# Run Camera Calibrator
#ros2 run camera_calibration cameracalibrator --size 10x7 --square 0.025 --ros-args -r image:=/oak/rgb/image_raw

# ros2_calib -> Calibration Software to LiDAR <-> Camera or LiDAR <-> LiDAR
#source /root/.venv/bin/activate
#ros2_calib