# Aeva API Sensor System Time #

### Overview of the program ###

This program demonstrates how to use Aeva API to change the sensor system time when the time protocol is HOST_SYSTEM_TIME

### Program dependencies ###

- Aeva API Debian package `Atlas_API_<VERSION>_<ARCH>.deb`  
- CMake (minimum version required 3.10)

### Steps to build and run the program ###

* Install the provided Aeva API Debian package on your system.
```bash
sudo dpkg -i Atlas_API_<VERSION>_<ARCH>.deb
```

* Create a `build` folder. The file structure should look like the following.
```
change_sensor_system_time/
├── build/
├── CMakeLists.txt
└── change_sensor_system_time.cc
```

* Run `cmake` and `make` commands inside the `build` folder.
```bash
mkdir build && cd build
cmake ..
make
```
This builds the target with the name `change_sensor_system_time` in the `build` directory.

* Run the program using the following commands. This program takes in the sensor IP address as the input.
```bash
./change_sensor_system_time SENSOR_IP
```
