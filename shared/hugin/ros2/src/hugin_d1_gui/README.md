Copyright (c) Sensrad 2023-2025

## hugin_d1_gui

This node (hugin_d1_gui) provides a GUI application for controlling the Hugin D1 radar. The hugin_d1_gui application is written in C++17 using gtkmm (gtk-3).

The hugin_d1_gui application supports basic operations such as:
start/stop Tx, make a time synchronization between ROS2 and radar time, select mode, and controlling some radar parameters.

## Requirements

Built and tested with ROS2 Jazzy Jalisco on Ubuntu 24.04. Install all dependencies by navigating into your ros2 workspace and run:
```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Build

Then build the node:
```bash
colcon build --packages-select hugin_d1_gui
```

Source the package:
```bash
source ~/ros2_ws/install/setup.bash
```

The hugin_d1_gui node is (standard usage) launched by
```bash
ros2 launch hugin_d1_gui hugin_d1_gui.launch.py
```
