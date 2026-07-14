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
    ros-$ROS_DISTRO-ros2-control

#Configure catkin workspace
ENV CATKIN_WS=/root/ros2_ws
RUN mkdir -p $CATKIN_WS/src

WORKDIR $CATKIN_WS/src
RUN git clone https://github.com/agilexrobotics/ugv_sdk.git
RUN git clone https://github.com/agilexrobotics/bunker_ros2.git

WORKDIR $CATKIN_WS/src/ugv_sdk

RUN git checkout -b working_commit c2f2aa0

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Clean-up
WORKDIR /root
RUN apt-get clean

CMD ["bash"]