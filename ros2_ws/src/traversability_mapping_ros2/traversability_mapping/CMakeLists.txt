cmake_minimum_required(VERSION 3.8)
project(traversability_mapping)

# ============================================================================

# C++ Standard

# ============================================================================

if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# ============================================================================

# Dependencies

# ============================================================================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)

find_package(elevation_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# ============================================================================

# Include Directories

# ============================================================================

include_directories(
include
${PCL_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

# ============================================================================

# traversability_filter

# ============================================================================

add_executable(
traversability_filter
src/traversability_filter.cpp
)

target_include_directories(
traversability_filter PUBLIC
include
)

ament_target_dependencies(
traversability_filter
rclcpp
tf2
tf2_ros
tf2_geometry_msgs
cv_bridge
image_transport
laser_geometry
pcl_conversions
pcl_ros
sensor_msgs
std_msgs
geometry_msgs
nav_msgs
visualization_msgs
elevation_msgs
)

target_link_libraries(
traversability_filter
${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
)

# ============================================================================

# traversability_map

# ============================================================================

add_executable(
traversability_map
src/traversability_map.cpp
)

target_include_directories(
traversability_map PUBLIC
include
)

ament_target_dependencies(
traversability_map
rclcpp
tf2
tf2_ros
tf2_geometry_msgs
pcl_conversions
pcl_ros
cv_bridge
image_transport
laser_geometry
sensor_msgs
std_msgs
geometry_msgs
nav_msgs
visualization_msgs
elevation_msgs
)

target_link_libraries(
traversability_map
${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
)

# ============================================================================

# traversability_prm

# ============================================================================

add_executable(
traversability_prm
src/traversability_prm.cpp
src/library/kdtree.c
)

target_include_directories(
traversability_prm PUBLIC
include
)

ament_target_dependencies(
traversability_prm
rclcpp
tf2
tf2_ros
tf2_geometry_msgs
cv_bridge
image_transport
laser_geometry
pcl_conversions
pcl_ros
sensor_msgs
std_msgs
geometry_msgs
nav_msgs
visualization_msgs
elevation_msgs
)

target_link_libraries(
traversability_prm
${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
)

# ============================================================================

# traversability_path

# ============================================================================

add_executable(
traversability_path
src/traversability_path.cpp
)

target_include_directories(
traversability_path PUBLIC
include
)

ament_target_dependencies(
traversability_path
rclcpp
tf2
tf2_ros
tf2_geometry_msgs
cv_bridge
image_transport
laser_geometry
pcl_conversions
pcl_ros
sensor_msgs
std_msgs
geometry_msgs
nav_msgs
visualization_msgs
elevation_msgs
)

target_link_libraries(
traversability_path
${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
)

# ============================================================================

# Planner Plugin Library

# ============================================================================

add_library(
tm_planner_lib SHARED
src/library/tm_planner.cpp
)

target_include_directories(
tm_planner_lib PUBLIC
include
)

ament_target_dependencies(
tm_planner_lib
rclcpp
pluginlib
nav2_core
tf2
tf2_ros
tf2_geometry_msgs
cv_bridge
image_transport
laser_geometry
pcl_conversions
pcl_ros
sensor_msgs
std_msgs
geometry_msgs
nav_msgs
elevation_msgs
)

target_link_libraries(
tm_planner_lib
${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
)

# ============================================================================

# Plugin Export

# ============================================================================

pluginlib_export_plugin_description_file(
nav2_core
tm_planner_plugin.xml
)

# ============================================================================

# Installation

# ============================================================================

install(
TARGETS
traversability_filter
traversability_map
traversability_prm
traversability_path
tm_planner_lib
DESTINATION lib/${PROJECT_NAME}
)

install(
DIRECTORY include/
DESTINATION include
)

install(
DIRECTORY launch/
DESTINATION share/${PROJECT_NAME}
)

install(
FILES tm_planner_plugin.xml
DESTINATION share/${PROJECT_NAME}
)

# ============================================================================

# Export

# ============================================================================

ament_export_include_directories(include)

ament_export_dependencies(
rclcpp
tf2
tf2_ros
tf2_geometry_msgs
pcl_conversions
pcl_ros
cv_bridge
image_transport
laser_geometry
sensor_msgs
std_msgs
geometry_msgs
nav_msgs
visualization_msgs
pluginlib
nav2_core
elevation_msgs
)

ament_package()
