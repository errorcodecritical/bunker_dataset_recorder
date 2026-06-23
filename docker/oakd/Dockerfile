# Use official ROS Noetic base image
FROM ros:noetic-ros-base

# Set environment variables to prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system and build dependencies
RUN apt-get update && apt-get install -y \
    git \
    libpcap-dev \
    libyaml-cpp-dev \
    python3-catkin-tools \
    libopencv-dev \
    ros-noetic-depthai-ros \
    && rm -rf /var/lib/apt/lists/*

# ... (Keep the rest of your Dockerfile/Entrypoint the same) ...

# Create a temporary space to resolve ROS dependencies
WORKDIR /tmp/ws/src
RUN git clone --recursive https://github.com/HesaiTechnology/HesaiLidar_General_ROS.git

# Install all ROS dependencies found in the package.xml
WORKDIR /tmp/ws
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /tmp/ws

# Create the main workspace directory
WORKDIR /catkin_ws

RUN echo "source /opt/ros/noetic/setup.bash"      >> /root/.bashrc && \     
    echo "source /catkin_ws/devel/setup.bash"     >> /root/.bashrc