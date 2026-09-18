from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('bunker_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    simulation = LaunchConfiguration('simulation')
    gui = LaunchConfiguration('gui')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )
    declare_simulation = DeclareLaunchArgument(
        'simulation', default_value='false',
        description='Pass simulation mode through to the xacro model',
    )
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='false',
        description='Launch joint_state_publisher_gui instead of joint_state_publisher',
    )

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'robot_state_publisher.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'simulation': simulation,
            'gui': gui,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'display.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_simulation,
        declare_gui,
        robot_state_publisher_launch,
        rviz_node,
    ])
