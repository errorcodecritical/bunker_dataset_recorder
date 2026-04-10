# Copyright (c) 2023-2025 Sensrad

cmake_minimum_required(VERSION 3.22)
project(oden)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(INTERNAL_BUILD "Enable internal-only features" ON)

# Require C++17
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(oden_interfaces REQUIRED)
find_package(raf2_interfaces REQUIRED)
find_package(liboden REQUIRED)

# Export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Define the executable
add_executable(oden src/main.cpp src/RadarPerception.cpp src/RadarPublisher.cpp)

# Define include directories
target_include_directories(oden PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Add common dependencies
ament_target_dependencies(
  oden
  liboden
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  Eigen3
  oden_interfaces
  raf2_interfaces)

target_link_libraries(oden Eigen3::Eigen)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# install configuration files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install targets
install(TARGETS oden DESTINATION lib/${PROJECT_NAME})

# Set TESTS flag to OFF by default
option(TESTS "Enable tests" OFF)

# Build tests
if(BUILD_TESTING AND TESTS)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()

ament_package()
