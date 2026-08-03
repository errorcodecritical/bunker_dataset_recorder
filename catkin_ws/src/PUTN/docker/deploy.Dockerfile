############################
# Base image
############################
FROM osrf/ros:melodic-desktop-full

LABEL maintainer="Afonso Carvalho"
LABEL description="PUTN + A-LOAM Deployment Environment"

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

############################
# System dependencies
############################
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    unzip \
    # Core libs
    libpcl-dev \
    libsuitesparse-dev \
    libgoogle-glog-dev \
    libatlas-base-dev \
    libeigen3-dev \
    # Python / catkin
    python-rosdep \
    python-catkin-tools \
    python-pip \
    python3-pip \
    python-tk \
    # ROS packages
    ros-melodic-cv-bridge \
    ros-melodic-tf \
    ros-melodic-image-transport \
    ros-melodic-message-filters \
    ros-melodic-pcl-ros \
    ros-melodic-ompl \
    ros-melodic-navigation \
    ros-melodic-velodyne \
    ros-melodic-velodyne-pointcloud \
    ros-melodic-velodyne-description \
    ros-melodic-rviz \
    ros-melodic-rviz-visual-tools \
    && rm -rf /var/lib/apt/lists/*

############################
# Python dependencies
############################
RUN pip install --no-cache-dir "casadi<3.6" && \
    pip3 install --no-cache-dir "casadi<3.6"

############################
# Install Ceres (A-LOAM requirement)
############################
ENV CERES_VERSION=1.12.0

RUN git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && \
    git checkout tags/${CERES_VERSION} && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && make install && \
    cd ../.. && rm -rf ceres-solver

############################
# ROS setup
############################
RUN rosdep init || true && rosdep update

RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

############################
# Catkin workspace
############################
WORKDIR /root/catkin_ws/src

# Clone PUTN
RUN git clone https://github.com/Forestry-Robotics-UC/PUTN.git

# Clone A-LOAM
RUN git clone https://github.com/Forestry-Robotics-UC/A-LOAM.git

############################
# Build workspace
############################
WORKDIR /root/catkin_ws

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
    catkin config --extend /opt/ros/melodic \
                 --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build"

############################
# Environment setup
############################
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc
RUN chmod +x /root/catkin_ws/src/PUTN/src/putn/putn_mpc/scripts/*
############################
# Default command
############################
WORKDIR /root/catkin_ws
CMD ["/bin/bash"]