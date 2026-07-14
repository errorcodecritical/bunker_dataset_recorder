#!/usr/bin/env python3
"""
ROS2 Launch file for Traversability Mapping - Online Mode

This launch file starts:
- TF static transforms
- Traversability filter, map, and PRM nodes
- Nav2 move_base with traversability planner
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    traversability_mapping_dir = get_package_share_directory('traversability_mapping')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # TF Static Transforms
    # These replace the ROS1 static_transform_publisher nodes
    tf_camera_init_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init_to_map',
        arguments=['0', '0', '0', '1.570795', '0', '1.570795', '/map', '/camera_init'],
        output='screen'
    )
    
    tf_base_link_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera',
        arguments=['0', '0', '0', '-1.570795', '-1.570795', '0', '/camera', '/base_link'],
        output='screen'
    )
    
    # Traversability Filter Node
    traversability_filter = Node(
        package='traversability_mapping',
        executable='traversability_filter',
        name='traversability_filter',
        output='screen'
    )
    
    # Traversability Map Node
    traversability_map = Node(
        package='traversability_mapping',
        executable='traversability_map',
        name='traversability_map',
        output='screen'
    )
    
    # Traversability PRM Node
    traversability_prm = Node(
        package='traversability_mapping',
        executable='traversability_prm',
        name='traversability_prm',
        output='screen'
    )
    
    # Note: traversability_path node is commented out in original
    # traversability_path = Node(
    #     package='traversability_mapping',
    #     executable='traversability_path',
    #     name='traversability_path',
    #     output='screen'
    # )
    
    # Include Nav2 (move_base equivalent in ROS2 is nav2)
    # For ROS2, we'll include the Nav2 bringup launch file
    # Note: In ROS2, the equivalent of move_base is nav2's navigation stack
    # You would typically use nav2_bringup for this
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # TF transforms
        tf_camera_init_to_map,
        tf_base_link_to_camera,
        
        # Traversability nodes
        traversability_filter,
        traversability_map,
        traversability_prm,
        # traversability_path,
        
        # Note: For Nav2 integration, you would typically include:
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory('nav2_bringup'), 
        #                      'launch', 'navigation.launch.py')
        #     ),
        #     launch_arguments={
        #         'use_sim_time': use_sim_time,
        #         'params_file': os.path.join(traversability_mapping_dir, 'launch', 'params', 'nav2_params.yaml')
        #     }.items()
        # )
    ])


if __name__ == '__main__':
    generate_launch_description()
