# Modify the Sensor SOME/IP Configurations #

### Overview of the program ###

This program demonstrates how to get and set the sensor network settings by changing the SOME/IP configurations.

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
change_someip_settings/
├── build/
├── CMakeLists.txt
└── change_someip_settings.cc
```

* Run `cmake` and `make` commands inside the `build` folder.
```bash
mkdir build && cd build
cmake ..
make
```
This builds the target with the name `change_someip_settings` in the `build` directory.

* Run the program using the following commands. This program takes the new SOME/IP configurations
```bash
./change_someip_settings CURRENT_IP SOMEIP_SERVICE_ID SOMEIP_INSTANCE_ID
```
