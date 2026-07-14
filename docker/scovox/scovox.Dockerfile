FROM ros:jazzy-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

#Install ROS Packages
RUN apt update && apt install -y ros-jazzy-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-cv-bridge

#Configure catkin workspace
ENV CATKIN_WS=/root/ros2_ws
RUN mkdir -p $CATKIN_WS/src

#Clone Scovox and explo_planner repo
WORKDIR $CATKIN_WS/src
RUN git clone https://github.com/kalhansb/scovox.git

RUN git clone https://github.com/kalhansb/explo_planner.git

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Clean-up
WORKDIR /root
RUN apt-get clean

CMD ["bash"]
