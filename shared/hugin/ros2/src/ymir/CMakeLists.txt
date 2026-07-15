# Copyright (c) 2025 Sensrad

cmake_minimum_required(VERSION 3.22)
project(ymir)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Require C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options to suppress CMake warnings
option(INTERNAL_BUILD "Enable internal-only features" ON)
option(BUILD_AS_ROS2_PACKAGE "Build as ROS2 package" ON)

# Find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(oden_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Build Ymir as a component library
add_library(ymir_component SHARED src/Ymir.cpp)

target_include_directories(ymir_component
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(
  ymir_component
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  visualization_msgs
  oden_interfaces
  tf2_ros
  tf2_geometry_msgs)

rclcpp_components_register_nodes(ymir_component ymir::Ymir)

# Create standalone executable for Ymir
add_executable(ymir_node src/ymir_main.cpp)
target_include_directories(ymir_node
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(ymir_node ymir_component)
ament_target_dependencies(ymir_node rclcpp)

# Install targets
install(TARGETS ymir_component)

install(TARGETS ymir_node DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Set TESTS flag to OFF by default
option(TESTS "Enable tests" OFF)

# Build tests
if(BUILD_TESTING AND TESTS)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # Ymir core logic tests - validates key fixes
  ament_add_gtest(${PROJECT_NAME}_core_test test/test_ymir_core.cpp)

  target_include_directories(${PROJECT_NAME}_core_test
                             PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

  ament_target_dependencies(${PROJECT_NAME}_core_test rclcpp tf2
                            tf2_geometry_msgs)
endif()

ament_package()
