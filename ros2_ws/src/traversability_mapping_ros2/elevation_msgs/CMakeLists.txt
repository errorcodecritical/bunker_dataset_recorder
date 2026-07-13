cmake_minimum_required(VERSION 3.8)
project(elevation_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(ament_lint_common REQUIRED)

# Find message dependencies
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Required for rosidl
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)

# Generate messages
set(MESSAGE_FILES
  msg/OccupancyElevation.msg
)

# Generate message headers
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MESSAGE_FILES}
  DEPENDENCIES geometry_msgs nav_msgs std_msgs sensor_msgs
)

# Export dependencies
ament_export_dependencies(
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

# Export include directories for message headers
ament_export_include_directories(
  include
)

# Build the package
ament_package()

