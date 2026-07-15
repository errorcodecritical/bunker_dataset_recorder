# Copyright (c) 2024-2025 Sensrad
cmake_minimum_required(VERSION 3.22)
project(raf2_interfaces)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(INTERNAL_BUILD "Enable internal-only features" ON)

# Silence CMake warnings for unused variables that may be passed from parent
if(DEFINED BUILD_AS_ROS2_PACKAGE)
  # Variable is defined but not used in this interface package
endif()
if(DEFINED TESTS)
  # Variable is defined but not used in this interface package
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# IDL generation
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files "msg/BinaryData.msg" "msg/Thresholds.msg" "msg/ControlState.msg"
              "msg/Header.msg" "msg/MetaData.msg")

set(srv_files
    "srv/RdrCtrlStartTx.srv" "srv/RdrCtrlStopTx.srv"
    "srv/RdrCtrlSetThresholds.srv" "srv/RdrCtrlSetActiveSeq.srv"
    "srv/SysCfgSetTime.srv" "srv/RdrSetSuperResolution.srv")

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} ${srv_files}
                           DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime)

# Build tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()

ament_package()
