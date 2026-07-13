FROM dustynv/ros:humble-desktop-l4t-r36.4.0

ENV DEBIAN_FRONTEND=noninteractive
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

SHELL ["/bin/bash", "-c"]

# 1. Fix Open Robotics Expired GPG Key Layer
RUN apt-get update || true && apt-get install -y curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros2-latest-archive-keyring.gpg

# 2. Install toolchains and core prerequisites
RUN apt-get update && apt-get install -y \
    curl \
    gpg \
    build-essential \
    cmake \
    git \
    libusb-1.0-0-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libboost-all-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# 3. Add the PPA repository
RUN curl -s https://koide3.github.io/ppa/setup_ppa.sh | bash

# 4. Install additional external structural library links
RUN apt-get update && apt-get install -y -o Dpkg::Options::="--force-overwrite" \
    libiridescence-dev \
    libglfw3-dev \
    libmetis-dev \
    libomp-dev \
    libfmt-dev \
    libspdlog-dev \
    libglm-dev \
    libpng-dev \
    libjpeg-dev \
    && rm -rf /var/lib/apt/lists/*

# 5. Build GTSAM from source
WORKDIR /tmp
RUN git clone https://github.com/borglab/gtsam.git && \
    cd gtsam && \
    git checkout 4.3a0 && \
    mkdir build && cd build && \
    cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_WITH_TBB=OFF \
      -DGTSAM_USE_SYSTEM_EIGEN=ON \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON && \
    make -j$(nproc) && \
    make install && \
    cd /tmp && rm -rf gtsam

# 6. Build gtsam_points from source (Crucial missing dependency)
RUN git clone https://github.com/koide3/gtsam_points.git && \
    cd gtsam_points && \
    mkdir build && cd build && \
    cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_WITH_CUDA=ON \
      -DCMAKE_CUDA_ARCHITECTURES=87 && \
    make -j$(nproc) && \
    make install && \
    cd /tmp && rm -rf gtsam_points

# 7. Build Workspace File Tree
RUN mkdir -p /glim_ws/src
WORKDIR /glim_ws/src

# 8. Clone GLIM and GLIM_ROS2 repositories recursively
RUN git clone --recursive https://github.com/koide3/glim.git && \
    git clone https://github.com/koide3/glim_ros2.git

# 9. Run rosdep and resolve definitions
WORKDIR /glim_ws
RUN if [ -f /opt/ros/humble/install/setup.bash ]; then source /opt/ros/humble/install/setup.bash; else source /opt/ros/humble/setup.bash; fi && \
    apt-get update && \
    rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    rm -rf /var/lib/apt/lists/*

# 10. Compile the workspace from source targeting Orin Ampere Cores (sm_87)
RUN if [ -f /opt/ros/humble/install/setup.bash ]; then source /opt/ros/humble/install/setup.bash; else source /opt/ros/humble/setup.bash; fi && \
    colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CUDA_ARCHITECTURES=87 \
    -DBUILD_WITH_CUDA=ON \
    -DBUILD_WITH_VIEWER=OFF \
    -DBUILD_WITH_MARCH_NATIVE=OFF

# 11. Finalize configuration hooks
RUN ldconfig
RUN echo "if [ -f /opt/ros/humble/install/setup.bash ]; then source /opt/ros/humble/install/setup.bash; else source /opt/ros/humble/setup.bash; fi" >> ~/.bashrc && \
    echo "source /glim_ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]