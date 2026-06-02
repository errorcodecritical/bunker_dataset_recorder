# Copyright 2016-2025 Aeva, Inc. All rights reserved.
# 
# Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
# code for terms and conditions that govern your use of this software. Any use, reproduction,
# disclosure, or distribution of this software and related documentation outside the terms of the
# EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

cmake_minimum_required(VERSION 3.10)

project(ros_aeva_publisher)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  aeva_msgs
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2_msgs
  visualization_msgs
)

catkin_package()

set(AEVA_API_ATLAS TRUE)
set(AEVA_API_BUILD_TYPE "ros")
set(AEVA_API_MIN_VERSION "3.1.0")

find_package(AevaAPI CONFIG PATHS "/opt/aeva/atlas-api/cmake" "/opt/aeva/atlas-api-cuda/cmake")

include_directories(
  ${AevaAPI_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${AevaAPI_LIBRARY_DIRS})

add_library(AevaPublisher src/AevaPublisher.cc src/AevaPublisher.h)

add_library(AevaAPICallbacks src/AevaAPICallbacks.cc src/AevaAPICallbacks.h)

add_executable(ros_aeva_publisher src/ros_aeva_publisher.cc)

target_link_libraries(ros_aeva_publisher
  ${catkin_LIBRARIES}
  AevaAPI
  AevaAPICallbacks
  AevaPublisher
)
