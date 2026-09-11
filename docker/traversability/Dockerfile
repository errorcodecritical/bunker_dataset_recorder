############################
# ARM64 Jetson base image  #
############################
FROM arm64v8/ros:melodic-ros-base-bionic AS base

LABEL maintainer="Arush Mendon"
LABEL contact="arush.mendon@isr.uc.pt"
LABEL description="Traversability Mapping Docker for NVIDIA Jetson ARM64"

ARG ROS_VERSION=melodic

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_VERSION}

SHELL ["/bin/bash", "-c"]


################################
# System and ROS dependencies  #
################################
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    curl \
    gnupg \
    lsb-release \
    ca-certificates \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    ros-${ROS_VERSION}-navigation \
    ros-${ROS_VERSION}-pcl-ros \
    ros-${ROS_VERSION}-pcl-conversions \
    ros-${ROS_VERSION}-tf \
    ros-${ROS_VERSION}-tf2 \
    ros-${ROS_VERSION}-tf2-ros \
    ros-${ROS_VERSION}-tf2-eigen \
    ros-${ROS_VERSION}-eigen-conversions \
    ros-${ROS_VERSION}-grid-map \
    ros-${ROS_VERSION}-message-generation \
    ros-${ROS_VERSION}-message-runtime \
    ros-${ROS_VERSION}-topic-tools \
    ros-${ROS_VERSION}-imu-filter-madgwick \
    libeigen3-dev \
    libpcl-dev \
    libtbb-dev \
    libboost-all-dev \
    libyaml-cpp-dev \
    libgoogle-glog-dev \
    libgflags-dev \
 && rm -rf /var/lib/apt/lists/*


#########################
# Initialise rosdep     #
#########################
RUN rosdep init 2>/dev/null || true && \
    rosdep update


#########################
# Install GTSAM 4.0     #
#########################
WORKDIR /root

RUN wget -O /tmp/gtsam.zip \
        https://github.com/borglab/gtsam/archive/refs/tags/4.0.0-alpha2.zip \
 && unzip /tmp/gtsam.zip -d /tmp \
 && mkdir -p /tmp/gtsam-4.0.0-alpha2/build \
 && cd /tmp/gtsam-4.0.0-alpha2/build \
 && cmake \
      -DCMAKE_BUILD_TYPE=Release \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_UNSTABLE=OFF \
      .. \
 && make -j"$(nproc)" \
 && make install \
 && ldconfig \
 && rm -rf /tmp/gtsam.zip /tmp/gtsam-4.0.0-alpha2


#########################
# Catkin workspace      #
#########################
RUN mkdir -p /root/catkin_ws/src

WORKDIR /root/catkin_ws

RUN source /opt/ros/${ROS_VERSION}/setup.bash && \
    catkin init


#########################
# Shell environment     #
#########################
RUN echo "source /opt/ros/${ROS_VERSION}/setup.bash" \
        >> /root/.bashrc && \
    echo 'if [ -f /root/catkin_ws/devel/setup.bash ]; then source /root/catkin_ws/devel/setup.bash; fi' \
        >> /root/.bashrc

WORKDIR /root/catkin_ws

CMD ["/bin/bash"]


###############################################################
# Development image
# Adds GUI and X11 support for RViz and graphical ROS tools
###############################################################
FROM base AS dev

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    x11-apps \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglu1-mesa \
    libx11-dev \
    libxext-dev \
    libxrender-dev \
    libgtk-3-dev \
    qt5-default \
    gedit \
    ros-${ROS_VERSION}-rviz \
    ros-${ROS_VERSION}-rqt \
    ros-${ROS_VERSION}-rqt-common-plugins \
 && rm -rf /var/lib/apt/lists/*


#########################
# Jetson GUI variables  #
#########################
ENV QT_X11_NO_MITSHM=1
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all


#########################
# Final environment     #
#########################

ENV DEBIAN_FRONTEND=dialog

# Create the main workspace directory
WORKDIR /catkin_ws

RUN echo "source /opt/ros/melodic/setup.bash"      >> /root/.bashrc && \     
    echo "source /catkin_ws/devel/setup.bash"     >> /root/.bashrc
