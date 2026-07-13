# Launch Files for Traversability Mapping ROS2

This directory contains ROS2 launch files for the traversability_mapping package.

## Launch Files

### 1. `online.launch.py`
**Purpose**: Run the complete traversability mapping system in online mode.

**Nodes Launched**:
- TF static transforms (camera_init_to_map, base_link_to_camera)
- `traversability_filter` - Filters and processes incoming point clouds
- `traversability_map` - Creates occupancy and elevation maps
- `traversability_prm` - PRM-based global planner

**Usage**:
```bash
ros2 launch traversability_mapping online.launch.py
```

**Note**: The original ROS1 version included `move_base` via an include file. In ROS2, you should use Nav2 instead. See the Nav2 integration section below.

---

### 2. `offline.launch.py`
**Purpose**: Run the traversability mapping system in offline mode with PCL crop box filtering.

**Nodes Launched**:
- PCL crop box filter (filters robot structure from point clouds)
- `traversability_filter` - Filters and processes incoming point clouds
- `traversability_map` - Creates occupancy and elevation maps
- `traversability_prm` - PRM-based global planner

**Crop Box Parameters**:
- min_x: -0.85
- max_x: 10.0
- min_y: -100.0
- max_y: 100.0
- min_z: 0.5
- max_z: 100.0

**Usage**:
```bash
ros2 launch traversability_mapping offline.launch.py
```

---

### 3. `offline_trav.launch.py`
**Purpose**: Run the traversability mapping system in offline mode with wider crop box bounds for traversability testing.

**Nodes Launched**:
- PCL crop box filter with wider bounds
- `traversability_filter` - Filters and processes incoming point clouds
- `traversability_map` - Creates occupancy and elevation maps
- `traversability_prm` - PRM-based global planner

**Crop Box Parameters**:
- min_x: -100.0
- max_x: 100.0
- min_y: -100.0
- max_y: 100.0
- min_z: -100.0
- max_z: 100.0

**Usage**:
```bash
ros2 launch traversability_mapping offline_trav.launch.py
```

---

### 4. `include/move_base.launch.py`
**Purpose**: Configure Nav2 with the traversability mapping planner.

**Important Note**: In ROS2, `move_base` from ROS1 has been replaced by the **Nav2 stack**. This launch file provides guidance on how to integrate the traversability planner with Nav2.

**Original ROS1 Configuration**:
The ROS1 version configured move_base with:
- Global planner: `tm_planner/TMPlanner`
- Local costmap parameters
- Global costmap parameters
- Move base parameters

**ROS2 Nav2 Integration**:
To use the traversability mapping planner with Nav2:

1. **Include Nav2 Bringup**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

launch_description = LaunchDescription([
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 
                         'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(
                get_package_share_directory('traversability_mapping'),
                'launch', 'params', 'nav2_params.yaml'
            ),
            'default_bt_xml_filename': os.path.join(
                get_package_share_directory('traversability_mapping'),
                'launch', 'params', 'nav2_bt_navigator.xml'
            )
        }.items()
    )
])
```

2. **Configure the Planner**:
In your Nav2 params file (e.g., `nav2_params.yaml`), configure the planner server to use the TMPlanner:

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["TMPlanner"]
    TMPlanner:
      plugin: "tm_planner::TMPlanner"
      tolerance: 0.5
```

3. **Configure Costmap to Use Traversability Map**:
In your costmap configuration, add a layer that subscribes to `/occupancy_map_local`:

```yaml
local_costmap:
  ros__parameters:
    update_frequency: 5.0
    layers:
      - inflation
      - traversability
    traversability:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_subscribe_transient_local: True
      topic: "/occupancy_map_local"
```

---

## Parameters

The `params/move_base` directory contains the original ROS1 parameter files:

- `global_costmap_params.yaml` - Global costmap configuration
- `local_costmap_params.yaml` - Local costmap configuration
- `move_base_params.yaml` - Move base configuration
- `local_costmap_params_trav.yaml` - Traversability-specific local costmap

**Note**: These are ROS1 format parameter files. For ROS2 Nav2, you will need to adapt them to the ROS2 YAML format and structure.

---

## Creating a Complete Navigation Launch File

Here's an example of a complete launch file that combines everything:

```python
#!/usr/bin/env python3
"""
Complete navigation launch file with traversability mapping
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    traversability_dir = get_package_share_directory('traversability_mapping')
    nav2_dir = get_package_share_directory('nav2_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        
        # TF static transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_init_to_map',
            arguments=['0', '0', '0', '1.570795', '0', '1.570795', '/map', '/camera_init'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera',
            arguments=['0', '0', '0', '-1.570795', '-1.570795', '0', '/camera', '/base_link'],
        ),
        
        # Traversability mapping nodes
        Node(
            package='traversability_mapping',
            executable='traversability_filter',
            name='traversability_filter',
            output='screen'
        ),
        Node(
            package='traversability_mapping',
            executable='traversability_map',
            name='traversability_map',
            output='screen'
        ),
        Node(
            package='traversability_mapping',
            executable='traversability_prm',
            name='traversability_prm',
            output='screen'
        ),
        
        # Nav2 navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': os.path.join(
                    traversability_dir, 
                    'launch', 'params', 'nav2_complete_params.yaml'
                ),
                'default_bt_xml_filename': os.path.join(
                    nav2_dir,
                    'behavior_trees', 'navigate_w_replanning_and_recovery.xml'
                )
            }.items()
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
```

---

## Important Notes

1. **ROS1 vs ROS2 Differences**:
   - ROS1 uses XML launch files (`*.launch`)
   - ROS2 uses Python launch files (`*.launch.py`)
   - ROS1 uses `move_base` package
   - ROS2 uses `nav2_bringup` and related packages

2. **Nodelet Replacement**:
   - The original ROS1 launch files used `nodelet` for PCL filtering
   - In ROS2, you can use standalone nodes from `pcl_ros` package
   - Alternatively, use ROS2 components

3. **Parameter Loading**:
   - ROS1: `<rosparam file="..." command="load"/>`
   - ROS2: Use the `parameters` argument in Node actions

4. **Include Files**:
   - ROS1: `<include file="$(find pkg)/path/to/file.launch"/>`
   - ROS2: `IncludeLaunchDescription(PythonLaunchDescriptionSource(...))`

---

## Testing the Launch Files

To test the launch files:

```bash
# Build the package first
colcon build --symlink-install
source install/setup.bash

# Run online mode
ros2 launch traversability_mapping online.launch.py

# Run offline mode
ros2 launch traversability_mapping offline.launch.py

# Run offline trav mode
ros2 launch traversability_mapping offline_trav.launch.py
```

---

## Customization

You can customize the launch files by:

1. **Adding/Removing Nodes**: Edit the launch file to add or remove nodes as needed
2. **Changing Parameters**: Modify the parameters in the launch file or create new parameter files
3. **Creating New Launch Files**: Copy an existing launch file and modify it for your use case
4. **Adding TF Transforms**: Add more static transforms as needed for your robot configuration

---

## Troubleshooting

**Issue**: Launch file not found
**Solution**: Make sure the launch file is in the `launch` directory and has the `.launch.py` extension

**Issue**: Module not found error
**Solution**: Make sure you have sourced the ROS2 setup file: `source /opt/ros/<distro>/setup.bash`

**Issue**: Package not found
**Solution**: Make sure the package is built and installed: `colcon build --symlink-install`

**Issue**: Nodes not starting
**Solution**: Check the console output for errors. Make sure all dependencies are installed.
