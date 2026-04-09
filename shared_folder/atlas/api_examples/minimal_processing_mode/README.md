# Aeva API Minimal Processing Mode #

### Overview of the program ###

Program to register callbacks and subscribe to the point group point cloud datastream using the minimal processing in the Aeva API.

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
minimal_processing_mode/
├── build/
├── CMakeLists.txt
└── minimal_processing_mode.cc
```

* Run `cmake` and `make` commands inside the `build` folder.
```bash
mkdir build && cd build
cmake ..
make
```
This builds the target with the name `minimal_processing_mode` in the `build` directory.

* Run the program using the following commands. This program takes in the sensor IP address as the input.
```bash
./minimal_processing_mode SENSOR_IP
```
