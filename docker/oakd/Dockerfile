ARG ARCH=linux/arm64/v8
FROM ros:jazzy-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt-get update \
    && apt-get install -y \
    libopencv-dev
    #ros2-testing-apt-source

# Install some python packages
RUN apt-get -y install \
    python3-serial

#Install ROS Packages
RUN apt-get update \
    && apt-get install -y ros-$ROS_DISTRO-depthai-ros \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp
    #Depthai-V3
    #ros-$ROS_DISTRO-depthai-ros-v3

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Clean-up
WORKDIR /root
RUN apt-get clean

CMD ["bash"]
