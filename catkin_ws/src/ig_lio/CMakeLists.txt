cmake_minimum_required(VERSION 3.0.2)
project(ig_lio)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  pcl_ros
)

# Optional: Livox support. Only enabled if the ROS 1 Livox driver is present in
# the workspace (it is NOT a hard dependency, so rosdep works without it). When
# found, HAVE_LIVOX is defined and the Livox CustomMsg path is compiled in.
find_package(livox_ros_driver QUIET)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(TBB REQUIRED)

include(cmake/glog.cmake)

# sophus
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gicp_lio
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include 
)

include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}	
)

add_executable(${PROJECT_NAME}_node 
  src/ig_lio_node
  src/pointcloud_preprocess.cpp
  src/lio.cpp
  src/voxel_map.cpp
  src/SymmetricEigenSolver.cpp
  src/timer.cpp
  src/faster_voxel_grid.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  glog
  gflags
  TBB::tbb
)

# Optional Livox support (see find_package above).
if(livox_ros_driver_FOUND)
  message(STATUS "ig_lio: livox_ros_driver found -- Livox support ENABLED (lidar_type: livox)")
  target_compile_definitions(${PROJECT_NAME}_node PRIVATE HAVE_LIVOX)
  target_include_directories(${PROJECT_NAME}_node SYSTEM PRIVATE ${livox_ros_driver_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME}_node ${livox_ros_driver_LIBRARIES})
  add_dependencies(${PROJECT_NAME}_node ${livox_ros_driver_EXPORTED_TARGETS})
else()
  message(STATUS "ig_lio: livox_ros_driver NOT found -- Livox support DISABLED. "
                 "Install it in your workspace and rebuild to enable lidar_type: livox.")
endif()

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE)
