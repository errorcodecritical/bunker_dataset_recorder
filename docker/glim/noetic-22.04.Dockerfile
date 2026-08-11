FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV LANG=en_US.UTF-8

# Base tools + Python 3.8 (Noetic's target Python, not Jammy's default 3.10)
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    build-essential \
    git \
    cmake \
    software-properties-common \
    python3-pip \
    python3-venv \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository -y ppa:deadsnakes/ppa \
    && apt-get update && apt-get install -y \
        python3.8 \
        python3.8-dev \
        python3.8-venv \
        python3.8-distutils \
    && rm -rf /var/lib/apt/lists/*

# Make python3.8 the default python3/python for this image
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1 \
    && curl -sS https://bootstrap.pypa.io/pip/3.8/get-pip.py | python3.8 \
    && ln -sf /usr/bin/python3.8 /usr/bin/python

RUN python3 -m pip install -U \
    rosdep \
    rosinstall_generator \
    wstool \
    rosinstall \
    vcstool \
    catkin_tools \
    empy==3.3.4

# wstool's bundled yaml.load() call breaks on PyYAML 5.1+ (missing Loader arg) - patch it
RUN sed -i 's/yaml.load(stream)/yaml.load(stream, Loader=yaml.SafeLoader)/' \
    /usr/local/lib/python3.8/dist-packages/wstool/config_yaml.py

RUN rosdep init && rosdep update --rosdistro noetic

# Fetch ROS Noetic CORE source tree only (ros_comm - no GUI/desktop/perception stack)
RUN mkdir -p /root/ros_catkin_ws/src
WORKDIR /root/ros_catkin_ws

RUN rosinstall_generator ros_comm --rosdistro noetic --deps --tar > noetic-ros_comm.rosinstall \
    && wstool init src noetic-ros_comm.rosinstall

RUN apt-get update && apt-get install -y \
    libboost-all-dev \
    libssl-dev \
    libbz2-dev \
    liblz4-dev \
    libpoco-dev \
    libtinyxml-dev \
    libtinyxml2-dev \
    libgtest-dev \
    libeigen3-dev \
    libgpgme-dev \
    libconsole-bridge-dev \
    liblog4cxx-dev \
    libapr1-dev \
    libaprutil1-dev \
    libpcre3-dev \
    uuid-dev \
    zlib1g-dev \
    libcurl4-openssl-dev \
    libyaml-cpp-dev

# Resolve dependencies against jammy
RUN rosdep install --from-paths src --ignore-src --rosdistro noetic -y \
    --skip-keys="python3-pykdl python3-sip-dev" \
    2>&1 | tee /root/rosdep_install.log

RUN python3 -m pip install catkin_pkg "pyyaml<5.1" empy==3.3.4

# Build
RUN ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_DISABLE_FIND_PACKAGE_log4cxx=ON -j$(nproc) || \
    ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_DISABLE_FIND_PACKAGE_log4cxx=ON

RUN echo "source /root/ros_catkin_ws/install_isolated/setup.bash" >> /root/.bashrc

# Workspace for your own packages
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws

SHELL ["/bin/bash", "-c"]
CMD ["/bin/bash"]