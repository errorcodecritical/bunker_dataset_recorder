ARG ARCH=linux/arm64/v8
FROM ros:jazzy-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install some python packages
RUN apt update \
    && apt install -y \
    python3-serial

#Install ROS Packages
RUN apt install -y ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp

#Configure catkin workspace
ENV CATKIN_WS=/root/ros2_ws
RUN mkdir -p $CATKIN_WS/src

#Clone Git repo
WORKDIR $CATKIN_WS/src
RUN git clone -b ros2 https://github.com/ros-drivers/nmea_navsat_driver.git

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Clean-up
WORKDIR /root
RUN apt-get clean

CMD ["bash"]