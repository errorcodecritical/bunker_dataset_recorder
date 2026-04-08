ARG ARCH=linux/arm64/v8
FROM ros:jazzy-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install some python packages
RUN apt update \
    && apt install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool \
    libboost-all-dev \
    libyaml-cpp-dev \
    unzip \
    wget

#Install ros2 pkg
RUN apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp

#Configure catkin workspace
ENV CATKIN_WS=/root/ros2_ws
RUN mkdir -p $CATKIN_WS/src

WORKDIR $CATKIN_WS/src
RUN git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
RUN rosdep update && rosdep install --from-paths . -y --ignore-src

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Clean-up
WORKDIR /root
RUN apt-get clean

CMD ["bash"]