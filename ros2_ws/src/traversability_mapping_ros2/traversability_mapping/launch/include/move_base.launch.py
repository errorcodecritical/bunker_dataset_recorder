#!/usr/bin/env python3
"""
ROS2 Launch file for Nav2 with Traversability Mapping Planner

This replaces the ROS1 move_base.launch file.
In ROS2, move_base is replaced by the Nav2 stack.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    traversability_mapping_dir = get_package_share_directory('traversability_mapping')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Load parameter files for Nav2
    global_costmap_params = os.path.join(
        traversability_mapping_dir, 
        'launch', 'params', 'move_base', 'global_costmap_params.yaml'
    )
    local_costmap_params = os.path.join(
        traversability_mapping_dir, 
        'launch', 'params', 'move_base', 'local_costmap_params.yaml'
    )
    move_base_params = os.path.join(
        traversability_mapping_dir, 
        'launch', 'params', 'move_base', 'move_base_params.yaml'
    )
    
    # Note: In ROS2, move_base is replaced by the Nav2 stack
    # The main components are:
    # - nav2_lifecycle_manager (replaces move_base)
    # - nav2_planner (global planner)
    # - nav2_controller (local planner)
    # - nav2_costmap_2d (costmap)
    # - etc.
    
    # For the traversability mapping integration, we need to:
    # 1. Load the tm_planner as a global planner plugin
    # 2. Configure Nav2 to use the traversability occupancy map
    
    # Nav2 Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl', 'bt_navigator', 'controller_server', 
                          'planner_server', 'recoveries_server', 'bt_navigator']},
        ]
    )
    
    # Note: In the original ROS1 version, move_base was configured with:
    # - base_global_planner: tm_planner/TMPlanner
    # In ROS2 Nav2, this would be configured in the params file
    
    # For now, we'll create a note about the configuration
    # The actual Nav2 configuration would be in a separate params file
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Nav2 Lifecycle Manager
        lifecycle_manager,
        
        # Note: The full Nav2 stack would typically be launched via:
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory('nav2_bringup'), 
        #                      'launch', 'navigation.launch.py')
        #     ),
        #     launch_arguments={
        #         'use_sim_time': use_sim_time,
        #         'params_file': os.path.join(traversability_mapping_dir, 'launch', 'params', 'nav2_params.yaml'),
        #         'default_bt_xml_filename': os.path.join(traversability_mapping_dir, 'launch', 'params', 'nav2_bt_navigator.xml')
        #     }.items()
        # )
        
        # The tm_planner would be configured in the Nav2 params file like:
        # planner_server:
        #   ros__parameters:
        #     expected_planner_frequency: 20.0
        #     planner_plugins: ["GridBased"]
        #     GridBased:
        #       plugin: "nav2_navfn_planner/NavfnPlanner"
        #       tolerance: 0.5
        #     # For traversability:
        #     # planner_plugins: ["TMPlanner"]
        #     # TMPlanner:
        #     #   plugin: "tm_planner::TMPlanner"
        #
    ])


if __name__ == '__main__':
    generate_launch_description()
