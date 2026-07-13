ARG BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0
FROM $BASE_IMAGE

ARG WITH_CUDA=ON
ENV WITH_CUDA=${WITH_CUDA}

ARG ROS_DISTRO=humble
ENV ROS_DISTRO=${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

# install ROS
RUN apt-get update \
  && apt-get install --no-install-recommends -y curl ca-certificates gpg-agent \
  && update-ca-certificates \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update \
  && apt-get install --no-install-recommends -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-${ROS_DISTRO}-ros-base \
    libfmt-dev libspdlog-dev libopencv-dev \
    software-properties-common ca-certificates \
    git cmake build-essential wget unzip libboost-all-dev \
    libmetis-dev libomp-dev \
    libpng-dev libjpeg-dev libeigen3-dev libglm-dev libglfw3-dev \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-tf2-ros \
    curl \
    gpg \
  # clean
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository ppa:koide3/iridescence && \
#    add-apt-repository ppa:koide3/gtsam && \
    apt-get update && \
    apt-get install -y libiridescence-dev
#libgtsam-no-tbb-dev



WORKDIR /root
RUN git clone https://github.com/koide3/gtsam.git
WORKDIR /root/gtsam/
RUN mkdir build
WORKDIR /root/gtsam/build
RUN cmake ..
#RUN make check
RUN make install


WORKDIR /root
RUN git clone https://github.com/koide3/gtsam_points
WORKDIR /root/gtsam_points/
RUN git checkout -b working_commit 85d0f4c
WORKDIR /root/gtsam_points/build
RUN cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_WITH_CUDA=${WITH_CUDA}
RUN make -j$(nproc)
RUN make install

# CLone glim repo
WORKDIR /root
RUN mkdir -p /root/ros2_ws/src

WORKDIR /root/ros2_ws/src

#RUN git clone https://github.com/koide3/glim_ext.git

RUN git clone https://github.com/koide3/glim_ros2.git

RUN git clone https://github.com/koide3/glim.git
WORKDIR /root/ros2_ws/src/glim
RUN git checkout -b working_commit 25ad190

# without viewer and CUDA
# WORKDIR /root/glim/build
# RUN cmake .. \
#   -DBUILD_WITH_CUDA=${WITH_CUDA} \
#   -DBUILD_WITH_VIEWER=OFF \
#   -DBUILD_WITH_MARCH_NATIVE=OFF \
#   -DCMAKE_BUILD_TYPE=Release
# RUN  make -j$(nproc)

# with viewer
# WORKDIR /root/glim/build
# RUN cmake .. \
#   -DBUILD_WITH_CUDA=${WITH_CUDA} \
#   -DBUILD_WITH_VIEWER=ON \
#   -DBUILD_WITH_MARCH_NATIVE=OFF \
#   -DCMAKE_BUILD_TYPE=Release
# RUN make -j$(nproc)

CMD ["bash"]
