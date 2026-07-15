Copyright (c) Sensrad 2023-2025


# runes

This is a ROS2 package for receiving perception data from `oden` and publishing them in a standardized format which can be visualized in e.g. rviz2 and foxglove-studio.


## Requirements

Built and tested with ROS2 Jazzy Jalisco on Ubuntu 24.04. Install all dependencies by navigating into your ros2 workspace and run:
```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
Rosdep might report an error on `oden_interfaces` part of the ros2 workspace. This shall be made available separately.

## Pre-requisites
- Availability of the `liboden` module.
- `oden_interfaces` module built and sourced.


## Build Instructions

* Add folder `runes/` and `oden/` to `src/` folder in ROS2 workspace.
* Change to ROS2 workspace.
* Build `runes` package:
```bash
colcon build --packages-select runes
```

* Source ROS2 workspace:
```bash
source install/setup.bash
```

Note that the ROS2 workspace shall be sourced in every active terminal window which interfaces with the ROS2 system (e.g., `ros2 bag record`).


## Usage Instructions

### Start

* Change to ROS2 workspace.

* Source ROS2 workspace:
```bash
source install/setup.bash
```

* Launch via launch-file:
```bash
ros2 launch runes runes.launch.py
```

This will start the `perception_visualizer` node manually.
