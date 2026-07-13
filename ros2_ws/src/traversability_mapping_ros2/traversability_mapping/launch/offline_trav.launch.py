#!/usr/bin/env python3
"""
ROS2 Launch file for Traversability Mapping - Offline Trav Mode

This launch file starts:
- PCL crop box filter with wider bounds
- Traversability filter, map, and PRM nodes
- Nav2 move_base with traversability planner
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    traversability_mapping_dir = get_package_share_directory('traversability_mapping')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # PCL Crop Box Filter with wider bounds for trav mode
    pcl_crop_box = Node(
        package='pcl_ros',
        executable='pcl_crop_box',
        name='crop_box_3',
        output='screen',
        parameters=[
            {'input': 'rslidar_points'},
            {'output': 'rslidar_points_filtered'},
            {'min_x': -100.0},
            {'max_x': 100.0},
            {'min_y': -100.0},
            {'max_y': 100.0},
            {'min_z': -100.0},
            {'max_z': 100.0},
            {'keep_organized': False},
            {'negative': False}
        ]
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
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # PCL Crop Box Filter
        pcl_crop_box,
        
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
