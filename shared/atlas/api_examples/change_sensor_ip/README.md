# Modify the Sensor Network Settings #

### Overview of the program ###

This program demonstrates how to get and set the sensor network settings by changing the sensor's IP address.

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
change_sensor_ip/
├── build/
├── CMakeLists.txt
└── change_sensor_ip.cc
```

* Run `cmake` and `make` commands inside the `build` folder.
```bash
mkdir build && cd build
cmake ..
make
```
This builds the target with the name `change_sensor_ip` in the `build` directory.

* Run the program using the following commands. This program takes the current and new sensor IP address.
```bash
./change_sensor_ip CURRENT_IP NEW_IP
```
