cmake_minimum_required(VERSION 3.17)
project(image_space_path_planning)

## Use C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_BUILD_TYPE MATCHES Debug)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pg")
endif()

## Find catkin and required ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
)

## Find system dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

find_package(CUDAToolkit REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs cv_bridge image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

# Build a library for the core code
add_library(image_space_path_planning_lib
  src/node_interface.cpp
  src/path_projector.cpp
  src/path_deformer.cpp
  src/path_optimizer.cpp
)

target_link_libraries(image_space_path_planning_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
)

# The main executable links to the library
add_executable(image_space_path_planning src/main.cpp)
target_link_libraries(image_space_path_planning image_space_path_planning_lib)

# Unit Testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GTest REQUIRED)
  catkin_add_gtest(test_path_projector test/test_path_projector.cpp)
  target_link_libraries(test_path_projector image_space_path_planning_lib ${catkin_LIBRARIES} GTest::GTest GTest::Main)
endif()