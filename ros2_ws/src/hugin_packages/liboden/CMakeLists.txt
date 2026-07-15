# Copyright (c) Sensrad 2023-2025
cmake_minimum_required(VERSION 3.8)
project(liboden)

# Find ROS dependencies
find_package(ament_cmake REQUIRED)

# Eigen3 is required
find_package(Eigen3 REQUIRED)

add_library(liboden SHARED IMPORTED)

# Set library name based on architecture
set(LIBODEN_FILENAME "liboden_${CMAKE_SYSTEM_PROCESSOR}.so")
# Check for supported architectures
if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|x86_64")
  message(FATAL_ERROR "Architecture not supported")
endif()

# set target library file
set_target_properties(
  liboden PROPERTIES IMPORTED_LOCATION
                     ${CMAKE_CURRENT_SOURCE_DIR}/lib/${LIBODEN_FILENAME})

# set target header files
set_target_properties(liboden PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                         ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Install headers (following the liboden source packet structure)
install(DIRECTORY include/ DESTINATION include)

# Install pre-compiled library
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/${LIBODEN_FILENAME}
        DESTINATION lib)

ament_export_libraries(liboden)
ament_export_include_directories(include)

ament_package()
