FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    python3-catkin-tools \
    python3-rosdep \
    libeigen3-dev \
    libpcl-dev \
    libtbb-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-eigen \
    ros-noetic-roscpp \
    ros-noetic-rospy \
    ros-noetic-roslib \
    ros-noetic-sensor-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-visualization-msgs \
    ros-noetic-std-msgs \
    ros-noetic-rviz \
    nano \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true && rosdep update

RUN mkdir -p /catkin_ws/src

WORKDIR /catkin_ws

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /catkin_ws/devel/setup.bash ]; then source /catkin_ws/devel/setup.bash; fi" >> /root/.bashrc

CMD ["bash"]
