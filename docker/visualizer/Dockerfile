ARG ARCH=linux/arm64/v8
FROM ros:jazzy-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt update \
    && apt install -y \
    libboost-all-dev \
    libncurses-dev \
    libgeographiclib-dev \
    libcxxopts-dev \
    libexpected-dev \
    libboost-dev \
    iproute2 \
    python3-venv \
    python3-pip \
    libxcb-cursor0

#Install ros2 pkg
RUN apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-rosbag2-storage-mcap \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-angles \
    ros-$ROS_DISTRO-topic-tools \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-backward-ros \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-vision-msgs \
    ros-$ROS_DISTRO-depthai-ros \
    ros-$ROS_DISTRO-camera-calibration
    #Depthai-V3
    #ros-$ROS_DISTRO-depthai-ros-v3


#Configure catkin workspace
ENV CATKIN_WS=/root/ros2_ws
RUN mkdir -p $CATKIN_WS/src

WORKDIR $CATKIN_WS/src
RUN git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
RUN rosdep update && rosdep install --from-paths . -y --ignore-src

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Clean-up
WORKDIR /root
RUN apt-get clean

# ros2_calib -> Calibration Software to LiDAR <-> Camera or LiDAR <-> LiDAR
RUN python3 -m venv .venv
RUN .venv/bin/pip install ros2_calib matplotlib

# ros2_calib need this fixes to the conditions about camera distortions, without this you can´t run the program
# Fix 1: Convert numpy.bool to Python bool
RUN sed -i 's/self.rectify_checkbox.setEnabled(has_distortion)/#self.rectify_checkbox.setEnabled(bool(has_distortion))/' /root/.venv/lib/python3.12/site-packages/ros2_calib/calibration_widget.py

# Fix 2: Handle missing intensity field
#RUN sed -i '735s/.*/        if hasattr(cloud_arr, "intensity"):\n            self.intensities = cloud_arr["intensity"]\n        else:\n            self.intensities = np.array([])/' /root/.venv/lib/python3.12/site-packages/ros2_calib/calibration_widget.py
RUN sed -i \
    -e '/if re_read_cloud:/,/self\.intensities = cloud_arr\["intensity"\]/{s/self\.intensities = cloud_arr\["intensity"\]/if "intensity" in cloud_arr.dtype.names:\n                self.intensities = cloud_arr["intensity"]\n            else:\n                self.intensities = np.array([])/}' \
    -e 's/self\.intensities_valid = self\.intensities\[self\.valid_indices\]/if len(self.intensities) > 0:\n            self.intensities_valid = self.intensities[self.valid_indices]\n        else:\n            self.intensities_valid = np.zeros(len(self.valid_indices))/' \
    /root/.venv/lib/python3.12/site-packages/ros2_calib/calibration_widget.py
    
# Fix 3: Check if intensities is not None before accessing .size
#RUN sed -i '739s/if self.intensities.size > 0:/if self.intensities is not None and self.intensities.size > 0:/' /root/.venv/lib/python3.12/site-packages/ros2_calib/calibration_widget.py

# Fix 4: Handle subscript access on intensities
#RUN sed -i '767s/self.intensities_valid = self.intensities\[self.valid_indices\]/self.intensities_valid = self.intensities\[self.valid_indices\] if self.intensities is not None else None/' /root/.venv/lib/python3.12/site-packages/ros2_calib/calibration_widget.py

CMD ["bash"]