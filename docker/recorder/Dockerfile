ARG ARCH=linux/arm64/v8
FROM ros:jazzy-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt update \
    && apt install -y \
    libboost-all-dev \
    libncurses-dev \
    libgeographiclib-dev \
    libcxxopts-dev \
    libexpected-dev \
    iproute2 \
    libboost-all-dev \
    libyaml-cpp-dev

#Install ros2 pkg
RUN apt install -y ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-angles \
    ros-jazzy-topic-tools \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-backward-ros \
    ros-jazzy-vision-msgs \
    ros-jazzy-rviz2

# Clone dependencies repos
WORKDIR /tmp/HesaiLidar_ROS_2.0
RUN git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git /tmp/HesaiLidar_ROS_2.0
RUN rosdep update && rosdep install --from-paths . -y --ignore-src

RUN apt-get clean

WORKDIR /ros2_ws

RUN echo "source /opt/ros/jazzy/setup.bash"      >> /root/.bashrc && \     
    echo "source /ros2_ws/install/setup.bash"     >> /root/.bashrc