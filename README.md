This repository provides a pipeline to record datasets with Agilex Bunker mini robot. For a detailed instruction manual, please check the [Wiki]()!

## 1. System Architecture

The entire data-acquisition system for the Bunker mini is organized under:

```
~/bunker_dataset_recorder/
├── Docker/
│   ├── atlas/
│   ├── bunker/
│   ├── emlid/
│   ├── foxglove/
│   ├── hesai/
│   ├── hugin/
│   ├── oakd/
│   ├── recorder/
│   ├── xsens/
│   └── docker-compose.yml
├── ros2_ws/
│   ├── bunker_description/
│   ├── bunker_gazebo_sim/
│   ├── atlas-build/
│   ├── bunker-build/
│   ├── curt-build/
│   ├── emlid-build/
│   ├── foxglove-build/
│   ├── hesai-build/
│   ├── hugin-build/
│   ├── oakd-build/
│   ├── recorder-build/
│   └── xsens-build/
├── shared_folder/
│   ├── atlas/
│   ├── hugin/
│   ├── atlas-launch.sh
│   ├── bunker-launch.sh
│   ├── emlid-launch.sh
│   ├── foxglove-launch.sh
│   ├── hesai-launch.sh
│   ├── hugin-launch.sh
│   ├── oakd-launch.sh
│   ├── recorder-launch.sh
│   └── xsens-launch.sh
├── sensor_configs/
│   ├── emlid/
│   ├── hesai/
│   ├── hugin/
│   ├── oakd/
│   └── xsens/
└── startup.sh
```

### 1.1 Docker Containers

Each sensor package has its own Dockerfile inside its corresponding directory:

- **atlas/** → Aeva Atlas 4D LiDAR driver
- **emlid/** → GNSS-RTK (Reach M2) driver
- **oakd/** → OAK-D camera driver
- **xsens/** → Xsens IMU
- **bunker/** → Bunker mini URDF
- **hesai/** → Hegin QT128 LiDAR
- **hugin/** → Sensrad Hugin D1 Radar
- **recorder/** → hector_recorder
- **foxglove/** → Foxglove and RViZ docker containers

A **docker-compose.yml** file creates all containers for the sensors and the recording.

### 1.2 Shared ROS 2 Workspace

The directory ```ros2_ws/``` is a workspace shared across all containers. Each container mounts:

- **ros2_ws/<sensor>-build/** → Build folder of each container This prevents each container from rebuilding the full workspace and allows faster startup. This directory also contains two packages that are being shared to the containers:

### 1.3 Shared Entry-Point Scripts

The folder **shared_folder/** contains launcher scripts used by each container:

- They source the ROS2 setup
- Build only the required packages
- And run the corresponding launch files or drivers

The recorder entry point runs the **hector_recorder** ROS2 command.

### 1.4 Sensor Configuration

This folder has every configuration needed to each sensor. Every sensor has its own directory and the files are linked to the respective containers.

### 1.5 Bunker mini URDF Package

This folder, inside ```ros2_ws/```, has the package needed to launch the Bunker mini URDF with all the sensors.

`ros2 launch bunker_description robot_state_publisher.launch.py`

### 1.6 Bunker Gazebo Sim

This package, located inside `ros2_ws/`, launches the Bunker URDF model into a Gazebo simulation environment.

To enable Gazebo simulation with ROS 2 control, the following changes have been made:

1. **`bunker.xacro`** – Updated to include the Gazebo plugin `libgz_ros2_control-system.so` and include the **ros2_control** parameters for all the wheels of the robot.

2. **Controller Configuration** – Created `bunker_controllers.yaml` inside `/bunker_gazebo_sim/config/`  
   This file defines the parameters for the wheel controllers and other necessary control logic.

#### Launch Package

The launch file (`bunker_gazebo_sim.launch.py`) starts:
- **`robot_state_publisher`** – Publishes the robot's TF transforms
- **`ros_gz_sim`** – Gazebo integration package for ROS 2 Jazzy

## 2. System Startup Procedure

### 2.1 Connecting to the CURT-NUC

The hotspot automatically powers on when the Bunker mini robot is turned on.

It hosts a Wi-Fi hotspot:

- **SSID:** fruc-bunker-jetson
- **IP Address:** 10.42.0.10
- **Connect via SSH:**
  ```
  ssh fruc-bunker-jetson@10.42.0.10
  ```

### 2.2 Launching the Recording System

Run the script **startup.sh** to start all the system components. The script will tranfer the RM3100 CAN device to the container and run:

1. **docker compose up -d**
   → Starts all sensor containers in the background
2. **docker compose run -i --rm recorder**
   → Opens an interactive shell and launches the hector_recorder TUI
   The recorder will then:

- Ask for a bag name (leave empty to auto-generate)
- Start recording once confirmed
  All ROS2 bags are saved outside the containers. In the directory that you mount to the recorder container, inside the docker-compose file. By default, ouside the repo main directory.

#### To close the system:

1. Press **Ctrl+C** to close the hector_recording and stop the system.
---

## 3. Recording Configuration

Current recording topics:

```text
/aeva/ATLAS/point_cloud_compensated /aeva/ATLAS/imu /aeva/ATLAS/odometry /aeva/ATLAS/point_cloud_metadata /lidar_packets /oak/rgb/image_raw /oak/rgb/camera_info /oak/rgb/image_rect /oak/stereo/image_raw /oak/stereo/camera_info /oak/imu/data /imu/data /imu/mag /heading /fix /tf /tf_static
```

To modify what is recorded:

1. Edit the **recorder entry-point script** in **shared_folder/recorder-launch.sh**
2. Update:
   - **TOPICS variable** → to add/remove ROS2 topics
   - **hector_recorder command** → configure:
     - Bag size limit
     - Storage format (MCAP, SQLite)
     - Compression
     - Performance parameters
     - etc…

---

## 4. Sensor Configuration

All sensor configuration live inside sensor_configs/.
Configuration files are located here:

### 4.1 Emlid

Config file:
`emlid/nmea_serial_driver.yaml`
Launch file:
`emlid/nmea_serial_driver.py`

### 4.2 Hesai QT128 LiDAR

Config file:

`hesai/config.yaml`

### 4.3 Sensrad Hugin D1 Radar

Config file:

`hugin/sensrad_params_1.yaml`

### 4.4 OAK-D Camera

Launch files:

`oakd/depthai_ros_driver/launch`

Config files:

`oakd/depthai_ros_driver/config`

### 4.5 Xsens IMU

Launch file:
`xsens/xsens_driver.launch.xml`

---
