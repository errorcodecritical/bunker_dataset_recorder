Copyright (c) Sensrad 2024-2025

# Hugin-D1

Here you find ROS2 nodes for working with the Hugin D1 radar. The
package provides two nodes: "control" and "parser". The control node
provides means to start and stop the radar as well as setting some
thresholds. The "control" node also publishes raw (binary) point-cloud
data which the "parser" node decodes into a pointcloud2 message.

## Network considerations

If you have performance problems you can try to increase the kernel
network buffer sizes (this is temporary until next reboot).

    sudo sysctl -w net.core.rmem_default=26214400
    sudo sysctl -w net.core.rmem_max=104857600
    sudo sysctl -w net.core.wmem_default=65536
    sudo sysctl -w net.core.wmem_max=104857600

For best performance and to ensure UDP packets are not dropped, add
the following lines to your /etc/sysctl.conf and reboot (the reboot is
required).

    net.core.rmem_default=26214400
    net.core.rmem_max=104857600
    net.core.wmem_default=65536
    net.core.wmem_max=104857600

## Requirements

Built and tested with ROS2 Jazzy Jalisco on Ubuntu 24.04. Install all
dependencies by navigating into your ros2 workspace and run:

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Build Instructions

- Add folder `hugin_d1` to `src/` folder in your ROS2 workspace.

- Change to ROS2 workspace.

- Build the `hugin_d1` set of packages:

```bash
colcon build --packages-select raf2_interfaces hugin_d1
```

- Source:

```bash
source install/setup.bash
```

Note that the ROS2 workspace shall be sourced in every active terminal
window which interfaces with the ROS2 system.

## Usage Instructions

### Start `hugin_d1` Package:

- Change to ROS2 workspace.

- Source ROS2 workspace:

```bash
source install/setup.bash
```

- Launch via launch-file:

```bash
ros2 launch hugin_d1 hugin_d1.launch.py
```

This will launch the necessary nodes to run the Hugin D1
radar. Parameters of the launch can be set in
`hugin_d1/config/hugin_d1.yaml`

### Record data
To record data from the radar, the following will capture the relevant topics into
a mcap bag:
ros2 bag record \
    /sensrad/radar_1/radar_data \
    /senarad/radar_1/radar_header \
    /sensrad/radar_1/control_state \
    /sensrad/radar_1/ethernet_logger \
    /sensrad/radar_1/meta_data \
    /tf_static \
    -s mcap -o bag_name


### Control Hugin Radar:

- Ensure the `hugin_d1` package is started.

- In a separate terminal, change to ROS2 workspace and source it.

- See below for how to start and stop the radar, write time to the
  radar, as well as setting threshold.

- Note also that the package hugin_d1_gui offers a GUI for radar
  control.

- In addition, the "yggdrasil" package provides functionality for
  simultaneous launch of ROS2 nodes for radar, GUI, perception and
  visualization.

(Below are examples when the Hugin D1 are launched with default namespace)

Start radar transmission:

```bash
ros2 service call /sensrad/radar_1/start_tx raf2_interfaces/srv/RdrCtrlStartTx
```

Stop radar transmission:

```bash
ros2 service call /sensrad/radar_1/stop_tx raf2_interfaces/srv/RdrCtrlStopTx
```

Sync radar to system (ROS2) time:

```bash
ros2 service call /sensrad/radar_1/set_time raf2_interfaces/srv/SysCfgSetTime "{'use_ros2_time': true}"
```

Set thresholds (example values):

```bash
ros2 service call /sensrad/radar_1/set_thresholds raf2_interfaces/srv/RdrCtrlSetThresholds "{static_threshold: 5.0, dynamic_azimuth: 15.0, dynamic_elevation: 5.0, dynamic_basic: 15.0}"
```

Change mode ("1" corresponds to ultra-long, and "2" corresponds to mid-mode):
```bash
ros2 service call /sensrad/radar_1/set_active_seq raf2_interfaces/srv/RdrCtrlSetActiveSeq "{requested_sequence_id: 1}"
```
