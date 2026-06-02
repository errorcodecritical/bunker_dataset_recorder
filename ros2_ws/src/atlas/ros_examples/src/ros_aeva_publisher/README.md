# Aeva ROS API Publisher #

### Overview of the program ###

This program demonstrates how to use ROS build of the Aeva API to publish ROS messages from the sensor.

### Program dependencies ###

- Aeva API Debian package `Atlas_API_<VERSION>_<ARCH>.deb`  
- ROS (recommended distribution ROS Noetic Ninjemys)  
- CMake (minimum version required 3.10)  
- catkin_tools

### Steps to build and run the program ###

* Install the provided Aeva API Debian package on your system.
```bash
sudo dpkg -i Atlas_API_<VERSION>_<ARCH>.deb
```

* __IMPORTANT:__ Please note that this example uses the ROS build of the Aeva API and not the standard Aeva API.



* Navigate to the `ros_examples` directory (which is our ROS workspace directory) and run the following command.
```bash
catkin build ros_aeva_publisher
```
This builds the ROS package with the name `ros_aeva_publisher`. It also creates a `build` and a `devel` folder inside the workspace directory.

* Source the `setup.bash` file from the `devel` directory.
```bash
source devel/setup.bash
```

* Run the program from the workspace directory. The program takes the sensor IP address and IDs as the inputs.
```bash
rosrun ros_aeva_publisher ros_aeva_publisher SENSOR_IP_0 SENSOR_NAME_0 [... SENSOR_IP_N SENSOR_NAME_N]
```
