cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit_controller)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roslaunch
    roscpp
    rospy
    geometry_msgs
    nav_msgs
    visualization_msgs
    actionlib
    actionlib_msgs
    tf
)

roslaunch_add_file_check(launch)

################################################################################
add_action_files(
  FILES
  FollowPath.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  actionlib_msgs
)
# Declare catkin configuration
catkin_package(
    CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf
    actionlib
    actionlib_msgs
    INCLUDE_DIRS include
    LIBRARIES PurePursuitController
)

################################################################################
# Build
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(PurePursuitController
    src/PurePursuitController.cpp
)

add_dependencies(PurePursuitController
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(PurePursuitController ${catkin_LIBRARIES})

add_executable(PurePursuitControllerNode
	src/PurePursuitControllerNode.cpp
)
target_link_libraries(PurePursuitControllerNode 
	PurePursuitController ${catkin_LIBRARIES}
)
add_dependencies(PurePursuitControllerNode PurePursuitController ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

################################################################################
# Install

catkin_install_python(PROGRAMS
  scripts/test_send_ref_path.py
  scripts/show_trajectory.py
  scripts/odom_rot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS PurePursuitController
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
    DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

