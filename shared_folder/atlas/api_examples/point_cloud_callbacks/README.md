# Register Point Cloud Callbacks #

### Overview of the program ###

This program demonstrates how to use Aeva API to subscribe and register callbacks for point cloud compensated, frame, and point group datastreams.

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
point_cloud_callbacks/
├── build/
├── CMakeLists.txt
└── point_cloud_callbacks.cc
```

* Run `cmake` and `make` commands inside the `build` folder.
```bash
mkdir build && cd build
cmake ..
make
```
This builds the target with the name `point_cloud_callbacks` in the `build` directory.

* Run the program using the following commands. This program takes in the sensor IP address and runtime duration (seconds).
```bash
./point_cloud_callbacks SENSOR_IP RUNTIME_DURATION_SECS
```
