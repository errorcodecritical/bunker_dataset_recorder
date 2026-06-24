# Use official ROS Noetic base image
FROM ros:noetic-ros-base

# Set environment variables to prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system and build dependencies
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    can-utils \
    iproute2 \
    libyaml-cpp-dev \
    libasio-dev \
    python3-catkin-tools \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# Create the main workspace directory
WORKDIR /catkin_ws

RUN echo "source /opt/ros/noetic/setup.bash"      >> /root/.bashrc && \     
    echo "source /catkin_ws/devel/setup.bash"     >> /root/.bashrc