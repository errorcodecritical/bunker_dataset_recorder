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
RUN apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-rosbag2-storage-mcap \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-angles \
    ros-$ROS_DISTRO-topic-tools \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-backward-ros \
    ros-$ROS_DISTRO-vision-msgs

#Configure catkin workspace
ENV CATKIN_WS=/root/ros2_ws
RUN mkdir -p $CATKIN_WS/src

WORKDIR $CATKIN_WS/src

# Clone dependencies repos
RUN git clone https://github.com/tu-darmstadt-ros-pkg/hector_recorder.git

RUN git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
RUN rosdep update && rosdep install --from-paths . -y --ignore-src


RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Clean-up
WORKDIR /root
RUN apt-get clean

CMD ["bash"]