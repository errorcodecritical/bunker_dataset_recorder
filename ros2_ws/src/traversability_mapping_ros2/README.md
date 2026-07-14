# Traversability Mapping ROS2 Port

This is a ROS2 port of the original ROS1 traversability_mapping package. It creates occupancy maps for uneven terrains like forests, suitable for use with Nav2 (Navigation Stack 2) in ROS2.

## Package Structure

- **elevation_msgs**: Contains custom message definitions for elevation maps
- **traversability_mapping**: Main package containing the traversability mapping algorithms

## Key Features

- Elevation mapping from point clouds
- Occupancy grid map generation
- Traversability calculation based on slope and height differences
- Dynamic map updating with Kalman filtering
- Integration with Nav2 via occupancy maps

## Dependencies

### ROS2 Packages
- rclcpp
- tf2_ros
- tf2_geometry_msgs
- sensor_msgs
- geometry_msgs
- nav_msgs
- nav2_core
- pluginlib
- cv_bridge
- image_transport
- pcl_ros
- pcl_conversions
- laser_geometry

### External Libraries
- PCL (Point Cloud Library)
- OpenCV
- Eigen3
- Boost

## Installation

1. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_url>
```

2. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Running the Traversability Mapping Node

```bash
ros2 run traversability_mapping traversability_map
```

### Running the Filter Node

```bash
ros2 run traversability_mapping traversability_filter
```

### Topics

#### Published Topics
- `/occupancy_map_local` (nav_msgs/msg/OccupancyGrid): Local occupancy grid map
- `/occupancy_map_local_height` (elevation_msgs/msg/OccupancyElevation): Occupancy map with elevation information
- `/elevation_pointcloud` (sensor_msgs/msg/PointCloud2): Elevation point cloud for visualization
- `/filtered_pointcloud` (sensor_msgs/msg/PointCloud2): Filtered point cloud

#### Subscribed Topics
- `/filtered_pointcloud` (sensor_msgs/msg/PointCloud2): Input filtered point cloud
- `/gps_waypoint_nav/odometry/navsat` (nav_msgs/msg/Odometry): Odometry information
- `/rslidar_points_filtered` (sensor_msgs/msg/PointCloud2): Raw point cloud input

## Integration with Nav2

The traversability_mapping package publishes a standard `nav_msgs/msg/OccupancyGrid` message on the `/occupancy_map_local` topic. This can be used by Nav2's costmap layers to incorporate traversability information into path planning.

To integrate with Nav2:

1. Ensure the traversability_map node is running
2. Configure Nav2's costmap to use the occupancy map from this package
3. The planner will automatically use the updated occupancy information

## Configuration

The main parameters are defined in `include/utility.h`. Key parameters include:

- `mapResolution`: Map resolution in meters (default: 0.1)
- `mapCubeLength`: Sub-map size in meters (default: 1.0)
- `localMapLength`: Size of the local occupancy map in meters (default: 25.0)
- `filterHeightLimit`: Maximum height difference for traversable terrain (default: 0.15)
- `filterAngleLimit`: Maximum slope angle in degrees (default: 20)
- `robotRadius`: Robot radius in meters (default: 0.30)

## Differences from ROS1 Version

1. **ROS2 Migration**: All ROS1 APIs have been replaced with ROS2 equivalents:
   - `ros::NodeHandle` → `rclcpp::Node`
   - `ros::Subscriber/Publisher` → `rclcpp::Subscription/Publisher`
   - `tf::TransformListener` → `tf2_ros::TransformListener`
   - Message types use ROS2 naming conventions (e.g., `nav_msgs::msg::OccupancyGrid`)

2. **Nav2 Integration**: The planner plugin now inherits from `nav2_core::GlobalPlanner` instead of `nav_core::BaseGlobalPlanner`

3. **Message Field Names**: Message fields use snake_case in ROS2 (e.g., `costMap` → `cost_map`)

4. **Time Handling**: Uses `rclcpp::Time` and `rclcpp::Duration` instead of `ros::Time` and `ros::Duration`

## Known Limitations

1. The filter, path, and prm nodes have simplified implementations. The main traversability_map node has the most complete port.
2. Some advanced features from the original package may not be fully implemented.
3. The planner plugin integration with Nav2 may require additional configuration.

## Future Work

- Complete port of all advanced filtering features
- Better integration with Nav2 costmap layers
- Performance optimization for real-time operation
- Support for different LiDAR sensor types
- ROS2 parameter server integration for runtime configuration

## License

The original package license applies (see original package.xml for details).

## Contributing

Contributions are welcome! Please open issues or pull requests for any bugs or enhancements.
