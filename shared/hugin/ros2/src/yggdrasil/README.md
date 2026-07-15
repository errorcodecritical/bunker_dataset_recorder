Copyright (c) Sensrad 2025


# yggdrasil

This is a ROS2 package for simultaneous launch of among others radar, perception, and visualization nodes.


## Requirements

Built and tested with ROS2 Jazzy Jalisco on Ubuntu 24.04. Install all dependencies by navigating into your ros2 workspace and run:
```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```


## Build Instructions

* Change to ROS2 workspace.
* Build and install the package:
```bash
colcon build --packages-select yggdrasil
```
To enable editing of configuration files without having to rebuild, we recommend building in the following manner:
```bash
colcon build --packages-select yggdrasil --symlink-install
```


* Source ROS2 workspace:
```bash
source install/setup.bash
```

Note that the ROS2 workspace shall be sourced in every active terminal window which interfaces with the ROS2 system (e.g., `ros2 bag record`).


## Usage Instructions

### Start nodes:

* Change to ROS2 workspace.

* Source ROS2 workspace:
```bash
source install/setup.bash
```

* Launch via launch-file:
```bash
ros2 launch yggdrasil sensrad.launch.py

```

This will start a collection of ROS2 nodes with parameter values specified by config/sensrad_params_1.yaml and extrinsic installation specified in extrinsics/extrinsics_hugin.yaml.

Note that the user can create custom launch description files by adding additional files in launch/profiles. Say for example that it is desirable to launch multiple radars. An example of this case is provided in launch/profiles/dual_radar.launch.py, which will launch two Sensrad node groups (Hugin D1, Hugin D1 GUI, Oden perception and Runes perception visualizer) together with a ROS2 visualizer (RViz2 or Foxglove Studio). To run the dual radar example, launch via

```bash
ros2 launch yggdrasil dual_radar.launch.py

```

The default configuration is that the nodes of one of the Sensrad groups are launched under the /sensrad_1 namespace, and the nodes of the other Sensrad group are launched under the /sensrad_2 namespace.
