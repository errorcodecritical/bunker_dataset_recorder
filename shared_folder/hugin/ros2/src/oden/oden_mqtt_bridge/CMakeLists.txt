# Copyright (c) 2025 Sensrad

cmake_minimum_required(VERSION 3.22)
project(oden_mqtt_bridge)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands by default
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

  target_compile_definitions(${target} PRIVATE BOOST_ASIO_NO_DEPRECATED)

endfunction()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Find custom interfaces
find_package(oden_interfaces REQUIRED)
find_package(raf2_interfaces REQUIRED)

find_package(PahoMqttCpp REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(oden_mqtt_bridge src/main.cpp src/OdenMqttBridge.cpp)
add_sensrad_compile_options(oden_mqtt_bridge)

# Add dependencies to target
ament_target_dependencies(oden_mqtt_bridge rclcpp std_msgs oden_interfaces
                          raf2_interfaces)

target_link_libraries(oden_mqtt_bridge nlohmann_json::nlohmann_json
                      PahoMqttCpp::paho-mqttpp3)

# Install target
install(TARGETS oden_mqtt_bridge DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Set TESTS flag to OFF by default
option(TESTS "Enable tests" OFF)

# Build tests
if(BUILD_TESTING AND TESTS)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()

ament_package()
