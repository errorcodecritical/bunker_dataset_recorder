ARG ARCH=linux/arm64/v8
FROM ros:jazzy-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install packages
# RUN apt update \
#     && apt install -y \
#     libboost-all-dev \
#     libncurses-dev \
#     libgeographiclib-dev \
#     libcxxopts-dev \
#     libexpected-dev \
#     iproute2 \
#     libboost-all-dev \
#     libyaml-cpp-dev

#Install ros2 pkg
RUN apt update \
    && apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-tf2-msgs \
    ros-$ROS_DISTRO-visualization-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-vision-msgs

RUN apt-get clean

WORKDIR /ros2_ws

RUN echo "source /opt/ros/jazzy/setup.bash"      >> /root/.bashrc && \     
    echo "source /ros2_ws/install/setup.bash"     >> /root/.bashrc