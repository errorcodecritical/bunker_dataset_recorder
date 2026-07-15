FROM ros:jazzy-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install some python packages
RUN apt update \
    && apt install -y \ 
    python3-serial

#Install ROS Packages
RUN apt-get install -y ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    ros-jazzy-pybind11-vendor \
    ros-jazzy-rmw-cyclonedds-cpp

#Configure catkin workspace
ENV CATKIN_WS=/root/ros2_ws
RUN mkdir -p $CATKIN_WS/src

#Clone Git repo
WORKDIR $CATKIN_WS/src
RUN git clone https://github.com/kalhansb/hmr_localisation.git
RUN git clone https://github.com/kalhansb/lidar_localization_ros2.git
#RUN git -C lidar_localization_ros2 checkout 0fe85a563b6d83641d09550d14cc4981ad0f5a97
RUN git clone -b humble https://github.com/rsasaki0109/ndt_omp_ros2.git
RUN git -C ndt_omp_ros2 checkout ef8a34985876359ecac7b7ad0004b6f409f6fbbc

RUN sed -i 's/kRegistrationSourceCloudKeepAliveCount = 4096;/kRegistrationSourceCloudKeepAliveCount = 4;/; s/kRegistrationTargetCloudKeepAliveCount = 4096;/kRegistrationTargetCloudKeepAliveCount = 4;/' $CATKIN_WS/src/lidar_localization_ros2/src/lidar_localization_component.cpp

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Clean-up
WORKDIR /root
RUN apt-get clean

CMD ["bash"]
