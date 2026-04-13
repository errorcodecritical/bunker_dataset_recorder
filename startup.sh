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
docker compose up -d --scale foxglove=0 &

sleep 1

# Open firefox in the foxglove site
#firefox  "https://app.foxglove.dev/isr/view?layoutId=lay_0e1Y5NtJWd6uyJvr&ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A9092" &

#sleep 1

docker compose run --rm -i recorder
