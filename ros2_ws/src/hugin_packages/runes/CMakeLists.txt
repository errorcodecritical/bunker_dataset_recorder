# Copyright (c) 2023-2025 Sensrad

cmake_minimum_required(VERSION 3.22)
project(runes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Require C++17
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(INTERNAL_BUILD "Enable internal-only features" ON)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(oden_interfaces REQUIRED)
find_package(message_filters REQUIRED)

find_package(Eigen3 3.3 REQUIRED)
find_package(liboden REQUIRED)

# Runes components (visualization)
add_library(
  runes_components SHARED src/PerceptionVisualizer.cpp src/PolygonGenerator.cpp
                          src/RegionOfInterestVisualization.cpp)

target_include_directories(runes_components
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(runes_components Eigen3::Eigen)

# Add common dependencies
ament_target_dependencies(
  runes_components
  liboden
  rclcpp
  std_msgs
  shape_msgs
  geometry_msgs
  visualization_msgs
  oden_interfaces
  message_filters
  Eigen3
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  rclcpp_components)

rclcpp_components_register_nodes(runes_components
                                 perception_visualizer::PerceptionVisualizer)

# Create standalone executable for perception visualizer
add_executable(perception_visualizer_node src/perception_visualizer_main.cpp)
target_link_libraries(perception_visualizer_node runes_components)
target_include_directories(perception_visualizer_node
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(perception_visualizer_node rclcpp)

# Install targets
install(TARGETS runes_components)

install(TARGETS perception_visualizer_node DESTINATION lib/${PROJECT_NAME})

# -----------------------------------------
# Install launch files
# -----------------------------------------
install(FILES runes_plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Set TESTS flag to OFF by default
option(TESTS "Enable tests" OFF)

# Build tests
if(BUILD_TESTING AND TESTS)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()

ament_package()
