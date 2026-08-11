# Use official OSRF ROS core image for Focal (Ubuntu 20.04) on arm64
FROM ros:noetic-ros-core

ARG ROS_DISTRO=noetic
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ca-certificates \
    gpg \
    wget

# Download Kitware's signing key and add it safely to your keyrings
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null \
    | gpg --dearmor - \
    | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null

# Add the Ubuntu 20.04 (focal) official Kitware repository
RUN echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' \
    | tee /etc/apt/sources.list.d/kitware.list >/dev/null

# Install dependencies, GTSAM, gtsam_points, and ROS packages in a single layer
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    cmake \
    git\
    curl \
    gnupg \
    ca-certificates \
    libfmt-dev \
    libspdlog-dev \
    libopencv-dev \
    libomp-dev \
    libeigen3-dev \
    libboost-all-dev \
    zenity \
    python3-catkin-tools \
    # ROS packages (ros-base and cv-bridge/tf2)
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-tf2-ros

# --- Install GTSAM ---
RUN git clone -b 4.2a9 https://github.com/borglab/gtsam /tmp/gtsam \
    && cd /tmp/gtsam \
    && mkdir build && cd build \
    && cmake .. \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    && make -j$(nproc) \
    && make install \
    && rm -rf /tmp/gtsam

# --- Install Iridescence ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        libglm-dev \
        libglfw3-dev \
        libpng-dev \
    && rm -rf /var/lib/apt/lists/* \
    && git clone --recursive https://github.com/koide3/iridescence /tmp/iridescence \
    && mkdir /tmp/iridescence/build && cd /tmp/iridescence/build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    && make -j$(nproc) \
    && make install \
    && rm -rf /tmp/iridescence

# --- Build gtsam_points ---
RUN git clone https://github.com/koide3/gtsam_points /tmp/gtsam_points \
    && cd /tmp/gtsam_points \
    && git fetch --tags --all \
    && git checkout v1.1.0 \
    && mkdir /tmp/gtsam_points/build && cd /tmp/gtsam_points/build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_DEMO=OFF \
        -DBUILD_TESTS=OFF \
        -DBUILD_TOOLS=OFF \
        -DBUILD_WITH_TBB=OFF \
        -DBUILD_WITH_OPENMP=ON \
        -DBUILD_WITH_CUDA=OFF \
        -DBUILD_WITH_CUDA_MULTIARCH=OFF \
        -DBUILD_WITH_MARCH_NATIVE=OFF \
        -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    && make -j$(nproc) \
    && make install \
    && rm -rf /tmp/gtsam_points \
    && ldconfig

# Set up the workspace
WORKDIR /catkin_ws

# Configure shell environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \     
    echo "source /catkin_ws/devel/setup.bash"       >> /root/.bashrc
