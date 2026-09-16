# Bunker Dataset Recorder (ROS 2)

A Dockerized, multi-sensor data-recording pipeline for the **Agilex Bunker Mini** UGV, built on **ROS 2 Jazzy**. Each sensor driver runs in its own container, all containers share a single ROS 2 workspace, and everything is orchestrated through Docker Compose and a single startup script.

> For a full step-by-step operating manual, see the project [Wiki]().

## Contents

- [Sensors & Hardware](#sensors--hardware)
- [System Architecture](#system-architecture)
- [Repository Layout](#repository-layout)
- [Prerequisites](#prerequisites)
- [Getting the Code](#getting-the-code)
- [Running the System](#running-the-system)
- [Recording Configuration](#recording-configuration)
- [Sensor Configuration Reference](#sensor-configuration-reference)
- [Bunker Description & Gazebo Simulation](#bunker-description--gazebo-simulation)

## Sensors & Hardware

| Sensor | Purpose | Container |
|---|---|---|
| Aeva Atlas | 4D FMCW LiDAR | `atlas` |
| Hesai QT128 | Mechanical LiDAR | `hesai` |
| Sensrad Hugin D1 | 4D imaging radar | `hugin` |
| OAK-D | Stereo camera (RGB, depth, IMU) | `oakd` |
| Xsens MTi | IMU | `xsens` |
| Emlid Reach M2 | GNSS-RTK | `emlid` |
| Agilex Bunker Mini | Robot base / chassis driver | `bunker_driver` |
| — | ROS 2 bag recording (`hector_recorder`) | `recorder` |
| — | RViz / visualization | `visualizer` |

## System Architecture

- **One container per sensor.** Each sensor has its own Dockerfile (`docker/<sensor>/Dockerfile`) and its own ROS 2 driver dependencies, so a failure or rebuild of one sensor doesn't affect the others.
- **Shared ROS 2 workspace.** All containers mount the same `ros2_ws/` directory, so interfaces (custom messages, description packages, drivers) are built once and available everywhere.
- **Config injection via bind mounts.** Sensor-specific tuning files in `config/` are bind-mounted directly over the driver's default config inside each container, so you can edit a YAML file on the host without rebuilding an image.
- **Central orchestration.** `docker/docker-compose.yml` defines every service, and `startup.sh` / `startup-nav.sh` bring the stack up with the correct set of containers enabled.
- **Recording.** The `recorder` container runs [`hector_recorder`](https://github.com/tu-darmstadt-ros-pkg/hector_recorder), an interactive TUI for starting/stopping `ros2 bag` recordings, writing bags outside the containers to a host-mounted directory.

## Repository Layout

```
bunker_dataset_recorder/
├── config/                     # Per-sensor configuration files, bind-mounted into containers
│   ├── bunker/                 #   twist_mux + teleop_twist_joy params
│   ├── emlid/                  #   nmea_navsat_driver node + config
│   ├── oakd/                   #   depthai_ros_driver launch/config overrides
│   └── xsens/                  #   norlab_xsens_driver node + launch overrides
├── docker/
│   ├── atlas/, bunker/, emlid/, hesai/, hugin/, oakd/, xsens/   # Per-sensor Dockerfiles
│   ├── recorder/                # hector_recorder image
│   ├── visualizer/              # RViz / GUI image
│   └── docker-compose.yml       # Defines all services
├── ros2_ws/
│   └── src/
│       ├── aeva_packages/         # aeva_msgs + ros2_aeva_publisher (Atlas LiDAR)
│       ├── bunker_packages/       # bunker_ros2 (submodule), ugv_sdk (submodule),
│       │                          #   bunker_description, bunker_gazebo_sim
│       ├── HesaiLidar_ROS_2.0/    # Hesai driver (submodule)
│       ├── hector_recorder/       # ROS 2 bag recording TUI (submodule)
│       ├── hugin_packages/        # hugin_d1, hugin_d1_gui, liboden, oden, runes,
│       │                          #   yggdrasil, ymir (Sensrad Hugin D1 radar stack)
│       ├── nmea_navsat_driver/    # GNSS driver (submodule)
│       └── norlab_xsens_driver/   # Xsens IMU driver (submodule)
├── shared/                     # Assets/scripts shared across containers (e.g. joystick setup,
│                                #   Atlas SDK .deb packages and API examples, Hugin sources)
├── startup.sh                  # Bring up the full recording stack
└── startup-nav.sh              # Bring up the stack for navigation (recorder/visualizer disabled)
```

### ROS 2 packages of note

- **`bunker_description`** — URDF/xacro model of the Bunker Mini with all sensors attached.
  ```bash
  ros2 launch bunker_description robot_state_publisher.launch.py
  ```
- **`bunker_gazebo_sim`** — Spawns the Bunker model in Gazebo with `ros2_control`-based wheel controllers, using the `libgz_ros2_control-system.so` plugin (see [Bunker Description & Gazebo Simulation](#bunker-description--gazebo-simulation)).

## Prerequisites

- Docker Engine + Docker Compose v2 (`docker compose ...`)
- NVIDIA Container Toolkit (required for the `hugin` and `visualizer` services, which request `runtime: nvidia`)
- Host running Linux with access to the required device nodes (`/dev`) for CAN, serial, USB, and camera devices
- A configured CAN interface (`can2`) at 500 kbit/s for the Bunker chassis driver

## Getting the Code

Several drivers are pulled in as git submodules, so clone recursively:

```bash
git clone --recurse-submodules -b ros2 https://github.com/errorcodecritical/bunker_dataset_recorder.git
cd bunker_dataset_recorder
```

If you already cloned without `--recurse-submodules`:

```bash
git submodule update --init --recursive
```

## Running the System

### 1. Connect to the robot's onboard computer

The Bunker Mini's onboard NUC hosts a Wi-Fi hotspot that powers on automatically with the robot:

- **SSID:** `fruc-bunker-jetson`
- **IP address:** `10.42.0.10`

```bash
ssh fruc-bunker-jetson@10.42.0.10
```

### 2. Start the stack

```bash
./startup.sh
```

This will:

1. Run `docker compose up -d` to start the sensor containers in the background.
2. Run `docker compose run -i --rm recorder` to attach an interactive `hector_recorder` session.

You'll be prompted for a bag name (leave blank to auto-generate one), then recording begins. All `ros2 bag` output is written to the host directory mounted in the `recorder`/`visualizer` services in `docker-compose.yml` (by default, a `rosbags/` folder outside the repository).

To bring the stack up **without** recording or visualization (e.g. for navigation/testing), use:

```bash
./startup-nav.sh
```

### 3. Stop the system

Press **Ctrl+C** in the recorder session. The trapped `EXIT` handler runs `docker compose down` to clean up all containers.

## Recording Configuration

Topics currently recorded by `hector_recorder`:

```text
/aeva/ATLAS/point_cloud_compensated
/aeva/ATLAS/imu
/aeva/ATLAS/odometry
/aeva/ATLAS/point_cloud_metadata
/lidar_packets
/oak/rgb/image_raw
/oak/rgb/camera_info
/oak/rgb/image_rect
/oak/stereo/image_raw
/oak/stereo/camera_info
/oak/imu/data
/imu/data
/imu/mag
/heading
/fix
/tf
/tf_static
```

To change what gets recorded:

1. Edit the recorder entry point (`shared/recorder-launch.sh`, used by the `recorder` service in `docker-compose.yml`).
2. Update:
   - The **`TOPICS`** variable — add or remove ROS 2 topics.
   - The **`hector_recorder`** invocation — bag size limit, storage format (MCAP/SQLite3), compression, and other performance parameters.

## Sensor Configuration Reference

All per-sensor tuning lives under `config/` and is bind-mounted into the matching container (see `docker/docker-compose.yml` for exact mount paths).

| Sensor | Files | Mounted into |
|---|---|---|
| **Emlid GNSS-RTK** | `config/emlid/nmea_serial_driver.yaml`, `config/emlid/nmea_serial_driver.py` | `nmea_navsat_driver` node/config |
| **Hesai QT128 LiDAR** | `config/hesai/config.yaml` *(referenced by the driver; see the `HesaiLidar_ROS_2.0` submodule for the full option set)* | Hesai driver container |
| **Sensrad Hugin D1 Radar** | `config/hugin/sensrad_params_1.yaml` *(see also `shared/hugin/ros2/src/yggdrasil/config/` and `extrinsics/`)* | Hugin driver container |
| **OAK-D Camera** | `config/oakd/depthai_ros_driver/launch/`, `config/oakd/depthai_ros_driver/config/` | `/opt/ros/jazzy/share/depthai_ros_driver/{launch,config}` |
| **Xsens IMU** | `config/xsens/xsens_driver.launch.xml`, `mtdevice.py`, `mtnode.py` | `norlab_xsens_driver` node/launch |
| **Bunker chassis** | `config/bunker/twist_mux.yaml`, `config/bunker/teleop_twist_joy.yaml` | `bunker_driver` container (`twist_mux`, `teleop_twist_joy`) |

Each sensor's ROS 2 driver environment uses:

```yaml
ROS_DOMAIN_ID: 30
RMW_IMPLEMENTATION: rmw_cyclonedds_cpp
CYCLONEDDS_URI: file:///shared/cyclonedds-config.xml
```

## Bunker Description & Gazebo Simulation

To simulate the Bunker Mini in Gazebo with `ros2_control`:

1. **`bunker.xacro`** includes the `libgz_ros2_control-system.so` Gazebo plugin and `ros2_control` parameters for all wheel joints.
2. **`bunker_controllers.yaml`** (in `ros2_ws/src/bunker_packages/bunker_gazebo_sim/config/`) defines the wheel controller and control-loop parameters.
3. **`bunker_gazebo_sim.launch.py`** starts:
   - `robot_state_publisher` — publishes the robot's TF tree
   - `ros_gz_sim` — the Gazebo/ROS 2 Jazzy integration bridge

```bash
ros2 launch bunker_gazebo_sim bunker_gazebo_sim.launch.py
```

---

### Submodules

| Package | Upstream |
|---|---|
| `bunker_ros2` | https://github.com/agilexrobotics/bunker_ros2 |
| `ugv_sdk` | https://github.com/agilexrobotics/ugv_sdk (branch `main`) |
| `HesaiLidar_ROS_2.0` | https://github.com/errorcodecritical/HesaiLidar_ROS_2.0 |
| `nmea_navsat_driver` | https://github.com/ros-drivers/nmea_navsat_driver (branch `ros2`) |
| `hector_recorder` | https://github.com/tu-darmstadt-ros-pkg/hector_recorder |
| `norlab_xsens_driver` | https://github.com/norlab-ulaval/norlab_xsens_driver |
