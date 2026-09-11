###############################################################
# ROS1 Noetic from source on Jetson AGX Orin / JetPack 6.1
# Base: L4T R36.4 / Ubuntu 22.04 / CUDA 12.6 / ARM64
###############################################################

FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN printf 'Acquire::ForceIPv4 "true";\nAcquire::Retries "5";\nAcquire::http::Timeout "60";\nAcquire::https::Timeout "60";\n' \
    > /etc/apt/apt.conf.d/99network-retries

RUN sed -i \
    's|http://ports.ubuntu.com/ubuntu-ports|https://ports.ubuntu.com/ubuntu-ports|g' \
    /etc/apt/sources.list

ENV ROS_DISTRO=noetic
ENV ROS_PYTHON_VERSION=3
ENV CUDA_HOME=/usr/local/cuda-12.6
ENV PATH=/usr/local/cuda-12.6/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:${LD_LIBRARY_PATH}
ENV TORCH_CUDA_ARCH_LIST="8.7"
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility

# Base packages and system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    ca-certificates \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    build-essential \
    cmake \
    git \
    pkg-config \
    nano \
    python3 \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-empy \
    python3-nose \
    python3-yaml \
    python3-numpy \
    python3-catkin-pkg \
    python3-rospkg \
    python3-rosinstall-generator \
    python3-vcstools \
    python3-rosdep2 \
    libboost-all-dev \
    libconsole-bridge-dev \
    libtinyxml2-dev \
    liburdfdom-dev \
    libeigen3-dev \
    libopencv-dev \
    libgoogle-glog-dev \
    libsuitesparse-dev \
    libatlas-base-dev \
    libpoco-dev \
    libgtest-dev \
    libbz2-dev \
    libgpgme-dev \
    libssl-dev \
    libyaml-cpp-dev \
    uuid-dev \
    liblog4cxx-dev \
    libapr1-dev \
    libaprutil1-dev \
    liblz4-dev \
    liborocos-kdl-dev \
    libopenblas-dev \
    libopenblas0-pthread \
    && rm -rf /var/lib/apt/lists/*

# CUDA 12.6 is already provided by l4t-jetpack:r36.4.0.
# Do not install cuda-compat-12-6 here; it depends on nvidia-l4t-core,
# which is supplied by the Jetson host/runtime stack, not this apt repo.
RUN nvcc --version && test -d /usr/local/cuda-12.6

# Python packages
RUN python3 -m pip install --upgrade pip setuptools wheel && \
    python3 -m pip install --no-cache-dir \
    rosdep \
    vcstool \
    mock \
    defusedxml \
    coverage \
    paramiko \
    netifaces \
    pycryptodomex \
    gnupg \
    numpy \
    scipy \
    matplotlib \
    PyYAML \
    rospkg \
    catkin_pkg \
    scikit-learn

# Jetson PyTorch for JetPack 6 / CUDA 12.6
RUN python3 -m pip install --no-cache-dir \
    torch==2.8.0 \
    torchvision==0.23.0 \
    --index-url=https://pypi.jetson-ai-lab.io/jp6/cu126

RUN python3 -m pip install --no-cache-dir --no-deps \
    timm \
    kornia \
    torchmetrics \
    pytorch-lightning

# rosdep
RUN rosdep init || true
RUN rosdep update || true

# Build ROS Noetic from source
WORKDIR /root/ros_catkin_ws

RUN rosinstall_generator \
    ros_comm \
    std_msgs \
    sensor_msgs \
    geometry_msgs \
    nav_msgs \
    tf \
    tf2 \
    tf2_ros \
    tf2_geometry_msgs \
    image_transport \
    cv_bridge \
    roscpp \
    rospy \
    --rosdistro noetic \
    --deps \
    --tar > noetic.rosinstall

RUN mkdir -p src && \
    vcs import --input noetic.rosinstall src

# Patch rosconsole for Ubuntu 22.04 / newer log4cxx
RUN sed -i 's/logger->addAppender(new ROSConsoleStdioAppender);/logger->addAppender(log4cxx::AppenderPtr(new ROSConsoleStdioAppender));/' \
    src/rosconsole/src/rosconsole/impl/rosconsole_log4cxx.cpp && \
    sed -i 's/return log4cxx::Logger::getLogger(name);/return static_cast<void*>(log4cxx::Logger::getLogger(name).get());/' \
    src/rosconsole/src/rosconsole/impl/rosconsole_log4cxx.cpp && \
    sed -i 's/log4cxx::spi::LoggerRepositoryPtr repo = log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME)->getLoggerRepository();/log4cxx::spi::LoggerRepositoryPtr repo = log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME)->getLoggerRepository().lock();/' \
    src/rosconsole/src/rosconsole/impl/rosconsole_log4cxx.cpp && \
    sed -i 's/logger->addAppender(g_log4cxx_appender);/logger->addAppender(log4cxx::AppenderPtr(g_log4cxx_appender));/' \
    src/rosconsole/src/rosconsole/impl/rosconsole_log4cxx.cpp && \
    sed -i 's/logger->removeAppender(g_log4cxx_appender);/logger->removeAppender(log4cxx::AppenderPtr(g_log4cxx_appender));/' \
    src/rosconsole/src/rosconsole/impl/rosconsole_log4cxx.cpp && \
    sed -i 's/log4cxx::Logger::getRootLogger()->getLoggerRepository()->shutdown();/log4cxx::Logger::getRootLogger()->getLoggerRepository().lock()->shutdown();/' \
    src/rosconsole/src/rosconsole/impl/rosconsole_log4cxx.cpp

RUN echo "Skipping rosdep install: Noetic rosdep rules target Ubuntu 20.04, while this JetPack 6 image is Ubuntu 22.04. System dependencies are installed explicitly above."

RUN ./src/catkin/bin/catkin_make_isolated \
    --install \
    --install-space /opt/ros/noetic \
    -j2 \
    -l2 \
    -DCMAKE_BUILD_TYPE=Release \
    -DCATKIN_ENABLE_TESTING=OFF \
    -DPYTHON_EXECUTABLE=/usr/bin/python3

# Build and install Ceres
WORKDIR /root
RUN git clone --branch 2.2.0 --depth 1 https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && \
    mkdir build && \
    cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DCERES_USE_CUDA=OFF && \
    make -j2 && \
    make install && \
    ldconfig && \
    cd /root && \
    rm -rf ceres-solver

# User catkin workspace
WORKDIR /root/catkin_ws/src
RUN source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_init_workspace

RUN pip install --no-cache-dir \
    numpy==1.26.4

WORKDIR /root/catkin_ws

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "export CUDA_HOME=/usr/local/cuda-12.6" >> /root/.bashrc && \
    echo "export PATH=/usr/local/cuda-12.6/bin:\$PATH" >> /root/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:/usr/local/cuda-12.6/compat:\$LD_LIBRARY_PATH" >> /root/.bashrc

# Create the main workspace directory
WORKDIR /catkin_ws

RUN echo "source /opt/ros/noetic/setup.bash"      >> /root/.bashrc && \     
    echo "source /catkin_ws/devel/setup.bash"     >> /root/.bashrc
