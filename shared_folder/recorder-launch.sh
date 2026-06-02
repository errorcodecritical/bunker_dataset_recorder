#!/bin/bash
set -e

#ROS 2 Middleware Implementation 

#Fastrtps
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

#Cyclonedds
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Install Atlas Eava ROS2 API
cd /root/shared_folder/atlas/
dpkg -i Atlas_AevaCLI_4_0_0_GA_aarch64.deb
dpkg --force-architecture -i Atlas_AevaAPI_4_0_0_GA_aarch64.deb

#Build workspace only with the packages descriminated on docker compose file
cd /root/ros2_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source /root/ros2_ws/install/setup.bash
sleep 10

#Database of random names to bags
names_dir="/root/shared_folder"
name_1=$(shuf -n 1 "$names_dir/.names_1")

# Directory where the bags are saved
BAG_DIR="/root/rosbags"

# Prompt user to give a bag name, otherwise give a random name from database
default_label="${name_1}"
if [ -t 0 ]; then
  read -r -p "Enter bag name (leave empty for default name): " user_label
else
  user_label=""
fi
if [ -z "$user_label" ]; then
  label="$default_label"
else
  label="$(echo "$user_label" | tr ' ' '_' | tr -cd '[:alnum:]_.-')"
fi
BAG_NAME="$(date +%Y_%m_%d_%H_%M_%S)__${label}_"

#Topics to record
#OAK: /oak/rgb/image_rect /oak/stereo/image_raw /oak/stereo/camera_info
#TOPICS="/aeva/ATLAS/point_cloud_compensated /aeva/ATLAS/imu /aeva/ATLAS/odometry /aeva/ATLAS/point_cloud_metadata /hesai/lidar_packets /hesai/points /sensrad/radar_1/radar_data /oak/rgb/image_raw /oak/rgb/camera_info /oak/imu/data /imu/data /imu/mag /heading /fix /tf /tf_static"
TOPICS="/aeva/ATLAS/health \
        /aeva/ATLAS/imu \
        /aeva/ATLAS/object_list/detections \
        /aeva/ATLAS/object_list/markers \
        /aeva/ATLAS/odometry \
        /aeva/ATLAS/point_cloud_compensated \
        /aeva/ATLAS/point_cloud_metadata \
        /aeva/ATLAS/twist \
        /diagnostics \
        /fix \
        /heading \
        /hesai/lidar_packets_loss \
        /hesai/lidar_packets \
        /imu/data \
        /imu/mag \
        /imu_data_str \
        /oak/imu/data \
        /oak/rgb/camera_info \
        /oak/rgb/image_rect/compressed \
        /sensrad/radar_1/binary_data \
        /sensrad/radar_1/control_state \
        /sensrad/radar_1/ethernet_logger \
        /sensrad/radar_1/meta_data \
        /sensrad/radar_1/oden/control_state \
        /sensrad/radar_1/oden/detections \
        /sensrad/radar_1/oden/ego_motion \
        /sensrad/radar_1/oden/extended_point_cloud \
        /sensrad/radar_1/oden/free_space_polygon \
        /sensrad/radar_1/oden/free_space_volume/camera_info \
        /sensrad/radar_1/oden/free_space_volume/image \
        /sensrad/radar_1/oden/free_space_volume/raw \
        /sensrad/radar_1/oden/ground_plane_data \
        /sensrad/radar_1/oden/local_map_cloud \
        /sensrad/radar_1/oden/object_tracks \
        /sensrad/radar_1/oden/occupancy_grid \
        /sensrad/radar_1/oden/region_of_interest \
        /sensrad/radar_1/oden/runes/aabb_object_tracks \
        /sensrad/radar_1/oden/runes/anomaly_voxels \
        /sensrad/radar_1/oden/runes/free_space_ground_polygon \
        /sensrad/radar_1/oden/runes/occupancy_grid_markers \
        /sensrad/radar_1/oden/runes/roi_marker \
        /sensrad/radar_1/oden/runes/static_map_voxels \
        /sensrad/radar_1/oden/static_map \
        /sensrad/radar_1/oden/statistics \
        /sensrad/radar_1/radar_data \
        /sensrad/radar_1/radar_header \
        /sensrad/ymir/ego_trajectory \
        /sensrad/ymir/ego_trajectory_markers \
        /tf \
        /tf_static"

mkdir -p "$BAG_DIR"

# Start ROS2 bag recording
echo "Starting ROS 2 bag recording..."
#Run hector_recorder
ros2 run hector_recorder record -o "$BAG_DIR/$BAG_NAME" --max-bag-size-gb 5 --topics $TOPICS &
BAG_PID=$!

trap "echo 'Stopping recording...'; kill -2 $BAG_PID 2>/dev/null || true" SIGINT SIGTERM

# Wait for recorder without aborting on non-zero (e.g., SIGINT=130)
set +e
wait $BAG_PID
REC_EXIT=$?
set -e
sync
echo "Rosbag stopped"

# Generate bag info file
bag_path=$(ls -1dt "$BAG_DIR/${BAG_NAME}"* 2>/dev/null | head -n1)
if [ -z "$bag_path" ]; then
 echo "No bag path found for ${BAG_NAME}"
 exit 1
fi
echo "Writing rosbag info to: $bag_path/info.txt"

#Wait briefly for metadata.yaml to appear (up to 30s)
for i in $(seq 1 30); do
 if [ -f "$bag_path/metadata.yaml" ]; then
   break
 fi
 sleep 1
done

#Generate info; don't abort the script if this fails
if ! ros2 bag info "$bag_path" > "$bag_path/info.txt"; then
 echo "ros2 bag info failed; info.txt not generated"
fi
sync
echo "Rosbag info saved"