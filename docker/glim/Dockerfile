ARG BASE_IMAGE=koide3/gtsam_points:focal
FROM $BASE_IMAGE

ARG ROS_DISTRO=noetic

# Remove the base image's broken Kitware sources BEFORE running apt-get update
RUN rm -f /etc/apt/sources.list.d/kitware.list* \
  && apt-get update \
  && apt-get upgrade -y \
  # install dependencies
  && apt-get install --no-install-recommends -y \
    curl libfmt-dev libspdlog-dev libopencv-dev zenity \
  # install ROS
  && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
  && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
  && apt-fast update \
  && apt-get install --no-install-recommends -y \
    libfmt-dev libspdlog-dev \
    ros-${ROS_DISTRO}-ros-base ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-tf2-ros \
    python3-catkin-tools \
  # clean
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# Re-add Kitware repos with signature verification bypassed
RUN echo 'deb [trusted=yes] https://apt.kitware.com/ubuntu/ focal main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null
RUN echo 'deb [trusted=yes] https://apt.kitware.com/ubuntu/ focal-rc main' | tee -a /etc/apt/sources.list.d/kitware.list >/dev/null

RUN apt-get update \
  && apt-get upgrade -y \
  && apt-fast clean \
  && rm -rf /var/lib/apt/lists/*
  
# Create the main workspace directory
WORKDIR /catkin_ws

RUN echo "source /opt/ros/noetic/setup.bash"      >> /root/.bashrc && \     
    echo "source /catkin_ws/devel/setup.bash"     >> /root/.bashrc