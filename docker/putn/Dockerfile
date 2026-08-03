# Use official ROS Noetic base image
FROM ros:noetic-ros-base

# Set environment variables to prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system and build dependencies
RUN apt-get update && apt-get install -y \
    git \
    libyaml-cpp-dev \
    python3-catkin-tools \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    unzip \
    # Core libs
    libpcl-dev \
    libsuitesparse-dev \
    libgoogle-glog-dev \
    libatlas-base-dev \
    libeigen3-dev \
    # Python / catkin
    python3-rosdep \
    python3-pip \
    python3-tk \
    # ROS packages
    ros-noetic-cv-bridge \
    ros-noetic-tf \
    ros-noetic-image-transport \
    ros-noetic-message-filters \
    ros-noetic-pcl-ros \
    ros-noetic-ompl \
    ros-noetic-navigation \
    ros-noetic-velodyne \
    ros-noetic-velodyne-pointcloud \
    ros-noetic-velodyne-description \
    ros-noetic-rviz \
    ros-noetic-rviz-visual-tools \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies
RUN pip3 install --no-cache-dir "casadi<3.6"

WORKDIR /tmp

# Install Ceres (A-LOAM requirement)
ENV CERES_VERSION=1.12.0

RUN git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && \
    git checkout tags/${CERES_VERSION} && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && make install && \
    cd ../.. && rm -rf ceres-solver

# Create the main workspace directory
WORKDIR /catkin_ws

RUN echo "source /opt/ros/noetic/setup.bash"      >> /root/.bashrc && \     
    echo "source /catkin_ws/devel/setup.bash"     >> /root/.bashrc