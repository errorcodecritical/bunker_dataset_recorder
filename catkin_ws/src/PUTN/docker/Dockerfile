######################
#     Base image     #
######################
FROM nvidia/cuda:12.1.0-runtime-ubuntu18.04 AS base

# Labels
LABEL maintainer="Afonso Carvalho"
LABEL contact="afonso.carvalho@isr.uc.pt"
LABEL description="PUTN + A-LOAM CUDA ROS Environment"

ARG ROS_VERSION="melodic"

# ==
# Disable dialog frontend
# ==
ARG DEBIAN_FRONTEND=noninteractive

# ==
# Select shell
# ==
SHELL ["/bin/bash", "-c"]

# ==
# Install ROS Melodic
# ==
RUN apt update && apt install -y --no-install-recommends \
    curl gnupg lsb-release build-essential cmake git wget unzip \
 && echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list \
 && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
 && apt update && apt install -y --no-install-recommends \
    ros-${ROS_VERSION}-desktop-full \
    python-rosdep python-catkin-tools python-rosinstall python-rosinstall-generator python-wstool \
 && rosdep init && rosdep update \
 && echo "source /opt/ros/${ROS_VERSION}/setup.bash" >> ~/.bashrc \
 && rm -rf /var/lib/apt/lists/*

# === Dependencies for PUTN ===
RUN apt-get update && apt-get install -y \
    # Core math & build deps
    libpcl-dev \
    libsuitesparse-dev \
    libgoogle-glog-dev \
    libatlas-base-dev \
    libeigen3-dev \
    # Core ROS libraries
    ros-${ROS_VERSION}-pcl-ros \
    ros-${ROS_VERSION}-cv-bridge \
    ros-${ROS_VERSION}-tf \
    ros-${ROS_VERSION}-image-transport \
    ros-${ROS_VERSION}-message-filters \
    # --- PUTN-specific packages (from README) ---
    ros-${ROS_VERSION}-ompl \
    ros-${ROS_VERSION}-robot-state-publisher \
    ros-${ROS_VERSION}-joint-state-controller \
    ros-${ROS_VERSION}-controller-interface \
    ros-${ROS_VERSION}-velocity-controllers \
    ros-${ROS_VERSION}-eigen-conversions \
    ros-${ROS_VERSION}-velodyne \
    ros-${ROS_VERSION}-rviz-visual-tools \
    ros-${ROS_VERSION}-velodyne-description \
    ros-${ROS_VERSION}-velodyne-gazebo-plugins \
    ros-${ROS_VERSION}-velodyne-pointcloud \
    ros-${ROS_VERSION}-xterm \
    ros-${ROS_VERSION}-python-tk \
 && rm -rf /var/lib/apt/lists/*

# Python dependencies (CasADi compatible with Ubuntu 18.04)
RUN apt-get update && apt-get install -y python-pip python3-pip && \
    pip install --no-cache-dir "casadi<3.6" && \
    pip3 install --no-cache-dir "casadi<3.6"

# --------------------------------------------------
# Install Ceres Solver
# --------------------------------------------------
ENV CERES_VERSION="1.12.0"
RUN git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && \
    git checkout tags/${CERES_VERSION} && \
    mkdir build && cd build && \
    cmake .. && make -j$(nproc) install && \
    cd ../.. && rm -rf ceres-solver

# ==
# Set up catkin workspace
# ==
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c "source /opt/ros/${ROS_VERSION}/setup.bash && cd /root/catkin_ws && catkin_init_workspace"

# ==
# Install navigation stack
# ==
RUN apt update && apt install -y \
    ros-${ROS_VERSION}-navigation \
 && rm -rf /var/lib/apt/lists/*

# ==
# Enable dialog again
# ==
ARG DEBIAN_FRONTEND=dialog

# ==
# Run bash
# ==
CMD ["/bin/bash"]
WORKDIR /root/catkin_ws


###############################################################
# Development image
# Adds GUI/X11 display support (for RViz, Sublime, etc.)
###############################################################
FROM base AS dev

ENV DEBIAN_FRONTEND=noninteractive

# Basic GUI tools (for visualization and editing)
RUN apt-get update && apt-get install -y \
    x11-apps \
    libgl1-mesa-glx \
    libx11-dev \
    libgtk-3-dev \
    qt5-default \
    gedit \
    rviz \
    ros-melodic-rqt \
 && rm -rf /var/lib/apt/lists/*

# Just set up the workspace directory — no build yet
RUN mkdir -p /root/catkin_ws/src

# Source setup files and env
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Default CMD
CMD ["/bin/bash"]
ENV DEBIAN_FRONTEND=dialog