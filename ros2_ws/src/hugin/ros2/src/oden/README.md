Copyright (c) Sensrad 2023-2025



# oden

The `oden` package in Sensrad's ROS2 suite is designed for advanced radar data processing, providing a range of perception functionalities to enhance the utility and accuracy of radar data.



### Key Functionalities

- **Odometry**: Determines the 3D velocity of the platform carrying the radar using doppler information. Oden also estimates the relative 6-degrees-of-freedom pose between subsequent frames.

- **Occupancy grid**: Computes a voxel representation of static objects. Each voxel includes an
occupancy probability.

- **Ground-Plane Estimation**: Computes the plane representing the ground in radar data.

- **Clustering and Detections**: Identifies and groups moving objects detected by the radar.

- **Free Space Estimation**: Analyzes radar data to estimate free space areas.

- **Local-Max Filtering**: Reduces the spread of the radar point cloud for more precise data.

- **Tracking of Dynamic Objects**: Estimates the 3D position and velocity and extent of objects which are in motion.

- **Classification of Dynamic Objects**: Estimates the type of tracked object, *i.e.* if it is a pedestrian, bicycle, car or truck.



### Published Topics

The `oden` node publishes several topics, each providing valuable data for perception and analysis:

- **ego_motion**: Outputs odometry information in the radar coordinate frame (x: right, y: forward, z: upward), along with boolean `is_valid` flags indicating the validity of the data. The data is grouped into velocity, rotation_rate, translation, and rotation:
  - velocity: 3D velocities in meters per second.
  - rotational_rate: pitch/roll/yaw-rates in radians per second.
  - translation: 3D position relative latest reset of Oden.
  - rotation_quat: quaternion describing the pose relative latest reset of Oden.
- **detections**: A list of clustered detections derived from the radar data, intended for use by tracking systems.
- **extended_point_cloud**: An enhanced PointCloud2 topic that includes the original point cloud data along with additional attributes:
  - `motion_status`: Indicates the motion state of each point (0 = static, 1 = unknown, 2 = dynamic).
  - `delta_velocity`: The radial velocity in meters per second, adjusted for ego-motion.
  - `available_for_tracker`: A flag indicating whether a point was used in the clustering algorithm.
  - `cluster_idx`: Integer values representing the cluster to which a point belongs. A value of "-1" denotes unclustered.
  - `distance_to_ground_plane`: Signed orthogonal distance to the estimated ground-plane.
  point or a static point.
  - `multi_path`: Boolean indicating if a point is due to multi-path.
  - `radar_cross_section`: Estimate of the radar cross section [dBsm]. Verified on corner reflectors (point targets).
  - `ghost_track`: Indicated whether a point is associated with a ghost track (multi-path and/or sidelobes)
  - `track_log_likelihood`: the log-likelihood of a point with respect to the closest mature track.
- **ground_plane_data**: Provides a model of the ground plane in the vicinity of the radar, described as a*x+b*y+c*z+d=0, along with a boolean `is_valid` flag.
- **occupancy grid**: A list with locations (in x,y,z), voxel size, and log-odds for voxels modelling the static part of the world. This feature is active only for perception configuration "Dynamic Default".
- **static map**: A list with locations (in x,y,z) and voxel size for voxels modelling the static part of the world. This feature is active only for perception configuration "Static Default". The main use-case for this feature is to perform change-detection for surveillance applications.
- **free_space**: A polygon representation of the estimated free space area.
- **object_tracks**: A list of tracked dynamic objects. Each object is tracked in the 3D Cartesian radar coordinate frame and is represented with a 3D position and a 3D velocity. The extent of the object is also estimated and modeled as a 3D ellipsoid which is represented by a 3x3 symmetric positive definite matrix.

These topics together offer essential data for effective radar-based perception and environmental understanding.
For details regarding the published data, please consult the message definitions in `oden_interfaces` and `liboden/include/liboden/IO_Types.hpp`.



### Configurable Parameters

The following ROS2 parameters can be adjusted either in the launch file or live:

- `radar_topic`: Topic name for radar point cloud data.
- `frame_id`: Frame identifier for the published extended point cloud.
- `vertical_height`: The vertical height of the radar installation above ground.
- `elevation_angle`: The elevation angle of the radar's boresight.
- `cluster_scale`: Scale factor on default clustering search radius.
- `free_space_max_range`: Max range for free-space evaluation.
- `free_space_lower_bound`: Lower bound on height relative the estimated ground plane for free-space check.
- `free_space_upper_bound`: Upper bound on height relative the estimated ground plane for free-space check.
- `stationary_updates`: Max number of updates of objects that have reached standstill
- `perception_config`: Perception mode; 0=default dynamic, 1=default static
- `region_of_interest`: Allows for defining which volume Oden shall process.

### Requirements

Built and tested with ROS2 Jazzy Jalisco on Ubuntu 24.04. Install all dependencies by navigating into your ros2 workspace and run:
```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
Rosdep might report an error on `oden_interfaces` part of the ros2 workspace. This shall be made available separately.

### Pre-requisites
- Availability of the `liboden` module.


## Install Instructions

To install and set up the `oden` package, follow these steps:

1. Build the package using colcon:

```bash
colcon build --packages-select oden
```

2. Source the installed packages to update your environment:

```bash
source install/setup.bash
```

3. To launch the radar perception functionality, use the provided launch file:

```bash
ros2 launch oden oden.launch.py
```
