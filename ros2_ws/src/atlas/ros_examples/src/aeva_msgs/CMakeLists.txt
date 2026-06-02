# Copyright 2016-2025 Aeva, Inc. All rights reserved.
# 
# Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
# code for terms and conditions that govern your use of this software. Any use, reproduction,
# disclosure, or distribution of this software and related documentation outside the terms of the
# EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

cmake_minimum_required(VERSION 3.10)
project(aeva_msgs)

# Compile as C++11, supported in ROS Kinetic and newer.
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  vision_msgs
)

# Generate messages in the 'msg' folder.
add_message_files(
  FILES
  Detection3D.msg
  Detection3DArray.msg
  EstimatedDetectionRangeBin.msg
  EstimatedDetectionRange.msg
  Health.msg
  PointCloudMetadata.msg
  RawData.msg
)

# Generate added messages and services with any dependencies listed here.
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  vision_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
