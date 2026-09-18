FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-domain-bridge \
      ros-jazzy-iceoryx-posh \
      iproute2

RUN apt-get clean

WORKDIR /ros2_ws

RUN echo "source /opt/ros/jazzy/setup.bash"      >> /root/.bashrc && \     
    echo "source /ros2_ws/install/setup.bash"     >> /root/.bashrc