# Copyright (c) 2025 Sensrad
#
# Dockerfile for launcher container to run ros2 nodes.
# It defines the ros2 workspace, installs dependencies, and builds the workspace.
#

# Location of ROS2 workspace:
ARG OVERLAY_WS=/opt/ros/overlay_ws

# Stage to install dependencies:
ARG ARCH=linux/arm64/v8
FROM ros:jazzy AS installer
ARG OVERLAY_WS

# Ensure keyrings are updated
RUN apt-get update && \
        apt-get install -y --no-install-recommends \
        ca-certificates gnupg dirmngr ubuntu-keyring && \
        rm -rf /var/lib/apt/lists/*

# Add ROS repo
RUN apt-get update && \
        apt-get install -y --no-install-recommends \
        ca-certificates ros2-apt-source && \
        rm -rf /var/lib/apt/lists/*

# Install ROS packages and dependencies:
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-rviz2 \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-tf-transformations \
        ros-jazzy-image-transport-plugins \
        ros-jazzy-image-proc \
        ros-jazzy-rosbag2-storage-mcap \
        ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
        libgtkmm-3.0-dev \
        # Hugin D1 GUI:
        libcanberra-gtk-module \
        libcanberra-gtk3-module \
        librsvg2-common \
        shared-mime-info \
        && rm -rf /var/lib/apt/lists/*


WORKDIR $OVERLAY_WS/src

# Copy ROS2 packages:
COPY shared_folder/hugin/ros2/src/ .

# Install ROS2 dependencies of ros2 nodes:
RUN apt-get update && \
        rosdep update && \
        apt-get install -y --no-install-recommends ros-jazzy-ament-cmake-clang-tidy && \
        rosdep install --from-paths . --ignore-src -r -y && \
        rm -rf /var/lib/apt/lists/*

# Change working directory:
WORKDIR ${OVERLAY_WS}

# Build ros2 workspace:
# hadolint ignore=SC1091
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --cmake-args \
        -DCMAKE_BUILD_TYPE=Release

# Source overlay:
RUN sed --in-place --expression \
"\$isource \"$OVERLAY_WS/install/setup.bash\"" \
/ros_entrypoint.sh

# Describe which ports are used normally by the application:
EXPOSE 6001/tcp
EXPOSE 6003/udp
EXPOSE 6007/tcp
EXPOSE 6010/tcp

# Modify the .bashrc to source the ROS2 workspace, and set the standard folder:
RUN mkdir ${OVERLAY_WS}/recordings && \
        echo "source \"$OVERLAY_WS/install/setup.bash\"" >> /root/.bashrc && \
        echo "cd $OVERLAY_WS/recordings" >> /root/.bashrc

# Setting to suppress errors in the GUI when run inside a container:
# Create the XDG_RUNTIME_DIR for user-specific runtime files:
RUN mkdir -p /run/user/1000 && chmod 700 /run/user/1000
ENV XDG_RUNTIME_DIR=/run/user/1000
# Disable accessibility features to prevent dbind warnings
ENV NO_AT_BRIDGE=1

# Run launch file
CMD ["bash"]
