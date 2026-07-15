FROM ros:jazzy-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
# ============================
# These are required for building PCL, OpenCV, Eigen, and other libraries
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    # PCL dependencies
    libpcl-dev \
    pcl-tools \
    \
    # OpenCV dependencies
    libopencv-dev \
    \
    # Eigen3 (required by PCL and other libraries)
    libeigen3-dev \
    \
    # Boost (required by some ROS2 packages)
    libboost-all-dev \
    \
    # FLANN (required by PCL)
    libflann-dev \
    \
    # Other utilities
    libyaml-cpp-dev \
    libssl-dev \
    lsb-release \
    && \
    rm -rf /var/lib/apt/lists/*

# Create workspace directory
# ==========================
RUN mkdir -p /root/ros2_ws/src
WORKDIR /ros2_ws

# Install additional ROS2 packages
# ================================
# These are the ROS2 dependencies needed by the package
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    # ROS2 core packages
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclcpp-components \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-visualization-msgs \
    \
    # TF2 packages
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    \
    # PCL ROS packages
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    \
    # Image and CV packages
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    \
    # Navigation packages
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    \
    # Laser geometry
    ros-${ROS_DISTRO}-laser-geometry \
    \
    # Pluginlib
    ros-${ROS_DISTRO}-pluginlib \
    \
    # Launch files
    ros-${ROS_DISTRO}-launch-ros \
    \
    # Other dependencies
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-flake8 \
    ros-${ROS_DISTRO}-ament-pep257 \
    ros-${ROS_DISTRO}-ament-cpplint \
    ros-${ROS_DISTRO}-ament-clang-format \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Clean-up
WORKDIR /root
RUN apt-get clean

CMD ["bash"]
