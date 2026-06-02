# Copyright 2016-2025 Aeva, Inc. All rights reserved.
# 
# Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
# code for terms and conditions that govern your use of this software. Any use, reproduction,
# disclosure, or distribution of this software and related documentation outside the terms of the
# EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

cmake_minimum_required(VERSION 3.10)

project(point_cloud_callbacks)

set(CMAKE_CXX_STANDARD 14)

set(AEVA_API_ATLAS TRUE)
set(AEVA_API_BUILD_TYPE "aeva")
set(AEVA_API_MIN_VERSION "1.0.0")

find_package(AevaAPI CONFIG PATHS "/opt/aeva/atlas-api/cmake" "/opt/aeva/atlas-api-cuda/cmake")

include_directories(${AevaAPI_INCLUDE_DIRS})

link_directories(${AevaAPI_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} point_cloud_callbacks.cc)

target_link_libraries(${PROJECT_NAME} AevaAPI)
