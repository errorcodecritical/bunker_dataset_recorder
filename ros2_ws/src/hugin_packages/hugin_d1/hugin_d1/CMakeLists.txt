# Copyright (c) 2024-2025 Sensrad

cmake_minimum_required(VERSION 3.22)
project(hugin_d1)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(INTERNAL_BUILD "Enable internal-only features" ON)

# Define a function that adds Sensrad's default compile options.
function(add_sensrad_compile_options target)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(
      ${target}
      PRIVATE -Wall
              -pedantic
              -Wextra
              -Wshadow
              -Wpointer-arith
              -Wcast-qual
              -Wcast-align
              -Wno-changes-meaning
              -Wstrict-aliasing=2)
  endif()
endfunction()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(raf2_interfaces REQUIRED)

# Boost
find_package(Boost REQUIRED COMPONENTS system)

# -----------------------------------------
# Control node
# -----------------------------------------
add_executable(
  control_node
  control/src/main.cpp
  control/src/ControlNode.cpp
  control/src/DebugCommands.cpp
  control/src/LogClient.cpp
  control/src/RTPReceiver.cpp
  control/src/TCPReceiver.cpp
  control/src/ArbeReceiver.cpp
  control/src/Hugin.cpp
  control/src/SomeIpBuilder.cpp
  control/src/SomeIpReader.cpp
  control/src/HumanReadableBytes.cpp)

add_sensrad_compile_options(control_node)

ament_target_dependencies(control_node rclcpp std_msgs raf2_interfaces
                          ament_index_cpp Boost)

target_include_directories(control_node
                           PRIVATE ${CMAKE_SOURCE_DIR}/control/include/)

# -----------------------------------------
# Parser node
# -----------------------------------------
add_executable(parser_node parser/src/main.cpp
                           parser/src/PointCloudParserNode.cpp)

target_compile_definitions(parser_node PRIVATE PLATFORM_OS_LINUX)

ament_target_dependencies(parser_node rclcpp std_msgs sensor_msgs
                          raf2_interfaces)

target_include_directories(parser_node
                           PRIVATE ${CMAKE_SOURCE_DIR}/parser/include/)

# -----------------------------------------
# Install launch and config files
# -----------------------------------------
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# -----------------------------------------
# Install executables
# -----------------------------------------

# Installs control node and parser node
install(TARGETS control_node parser_node DESTINATION lib/${PROJECT_NAME})

# Installs the flash utility into the ros2 package, this is used for getting
# production data and debug.

# Detect architecture
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(FLASH_BINARY bin/sensrad-flash-arm64)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(FLASH_BINARY bin/sensrad-flash-x86_64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Install it under the common name
install(
  PROGRAMS ${FLASH_BINARY}
  DESTINATION lib/${PROJECT_NAME}
  RENAME sensrad-flash)

# -----------------------------------------
# Tests
# -----------------------------------------

# Set TESTS flag to OFF by default
option(TESTS "Enable tests" OFF)

# Build tests
if(BUILD_TESTING AND TESTS)
  find_package(Eigen3 REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # Thresholds
  ament_add_gtest(${PROJECT_NAME}_thresholds_test tests/test_thresholds.cpp)

  target_include_directories(
    ${PROJECT_NAME}_thresholds_test
    PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/control/include>
            $<INSTALL_INTERFACE:include>)

  # Pointcloud
  ament_add_gtest(${PROJECT_NAME}_pointcloud_test tests/test_pointcloud.cpp
                  tests/read_test_files.cpp)

  target_link_libraries(${PROJECT_NAME}_pointcloud_test Eigen3::Eigen)

  target_include_directories(
    ${PROJECT_NAME}_pointcloud_test
    PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/parser/include>
            $<INSTALL_INTERFACE:include>)

  # Set working directory so that the pointcloud test can find the test files.
  set_tests_properties(
    ${PROJECT_NAME}_pointcloud_test
    PROPERTIES WORKING_DIRECTORY
               ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_resources)

endif()

ament_package()
