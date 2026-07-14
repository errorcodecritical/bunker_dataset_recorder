#!/usr/bin/env bash

# Exit if any command fails
set -e

# To use GUI uncomment the following line
#xhost +

# Function to cleanup: move can0 back to host and rename to can2
cleanup() {

    local exit_code=$?

    docker compose down

    return $exit_code
}

# Trap EXIT signal to ensure cleanup runs
trap cleanup EXIT

# Bring up the network connection (if needed)
#sudo nmcli connection up Hesai
#sudo nmcli connection up Aeva Atlas
#sudo nmcli connection up Hugin

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
cd $SCRIPT_DIR/docker

# Start your container in detached mode
#Run foxglove to either start the foxglove server or the rviz -> Change what you want inside foxglove-launcher.sh
docker compose up --scale recorder=0 --scale visualizer=0 --scale traversability_ros2=0 --scale nav2=0 --scale glim=0 --scale atlas=0 --scale hugin=0 &

wait
