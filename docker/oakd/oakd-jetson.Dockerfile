ARG ARCH=linux/arm64/v8
FROM ros:jazzy-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt-get update \
    && apt-get install -y \
    libopencv-dev

# Install some python packages
RUN apt-get -y install \
    python3-serial

#Install ROS Packages
RUN apt-get install -y ros-$ROS_DISTRO-depthai-ros

#Configure catkin workspace
ENV CATKIN_WS=/root/ros2_ws
RUN mkdir -p $CATKIN_WS/src

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Clean-up
WORKDIR /root
RUN apt-get clean

CMD ["bash"]
