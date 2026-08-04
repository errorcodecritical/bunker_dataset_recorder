# Agilex Bunker Robot Description Package

ROS Noetic package for launching the Agilex Bunker robot URDF description.

## Package Contents

- **URDF Files**: Robot description in Xacro format
- **Launch Files**: ROS 1 launch files for different use cases
- **Configuration Files**: Gazebo/RViz configuration files
- **Meshes**: STL, OBJ, and GLTF mesh files for visualization
- **RViz Config**: Pre-configured RViz visualization settings

## Launch Files

### 1. `robot_state_publisher.launch`
Launches the robot state publisher with optional joint state publisher GUI.

```bash
roslaunch bunker_description robot_state_publisher.launch
```

**Arguments:**
- `use_sim_time` (default: false) - Use Gazebo simulation time
- `simulation` (default: false) - Pass simulation mode to Xacro
- `gui` (default: false) - Enable `joint_state_publisher_gui`

### 2. `view_robot.launch`
Launches robot state publisher with RViz visualization.

```bash
roslaunch bunker_description view_robot.launch
```

**Arguments:**
- `use_sim_time` (default: false)
- `simulation` (default: false)
- `gui` (default: false)

### 3. `robot_description.launch`
Basic launch file for the robot state publisher only.

```bash
roslaunch bunker_description robot_description.launch
```

## Installation

1. Navigate to your ROS workspace:
```bash
cd ~/catkin_ws
```

2. Build the package:
```bash
catkin_make --pkg bunker_description
source devel/setup.bash
```

## Usage Examples

### View the robot in RViz
```bash
roslaunch bunker_description view_robot.launch gui:=true
```

### Control joint states with GUI
```bash
roslaunch bunker_description robot_state_publisher.launch gui:=true
```

### Use with Gazebo simulation
```bash
roslaunch bunker_description robot_state_publisher.launch simulation:=true use_sim_time:=true
```

## Dependencies

The package depends on:
- `robot_state_publisher`
- `joint_state_publisher`
- `joint_state_publisher_gui`
- `xacro`
- `rviz`

## Customization

### Modify Robot Inertia
Edit `urdf/bunker.urdf.xacro` to adjust inertial properties.

### Update Sensor Transforms
Modify the joint origins in `urdf/bunker.urdf.xacro` or `urdf/sensors.urdf.xacro`.

### Adjust RViz Display
Edit `config/rviz.rviz` to customize visualization settings.

## Troubleshooting

### Meshes not loading
Ensure the package is built and sourced:
```bash
catkin_make --pkg bunker_description
source devel/setup.bash
```

### Xacro variables not resolved
Check that all referenced properties in your Xacro files are defined correctly.

### TF tree not updating
Verify `joint_state_publisher` is running and publishing on `/joint_states`.

## Package Info

- **Robot Name**: agilex_bunker_mini
- **Maintainer**: [Update in package.xml]
- **License**: [Update in package.xml]
- **Version**: 0.0.1

## Notes

- The package now targets ROS Noetic / ROS 1
- Xacro files are processed at launch for flexibility
- RViz is launched using ROS 1 `rviz`
