ARG ARCH=linux/arm64/v8
FROM ros:humble-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

#Install System Packages
RUN apt update && apt install -y \
    libasio-dev

#Install ROS Packages
RUN apt update && apt install -y \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-twist-mux \
    ros-$ROS_DISTRO-ros2-control

RUN apt-get clean

WORKDIR /ros2_ws

RUN echo "source /opt/ros/jazzy/setup.bash"      >> /root/.bashrc && \     
    echo "source /ros2_ws/install/setup.bash"     >> /root/.bashrc