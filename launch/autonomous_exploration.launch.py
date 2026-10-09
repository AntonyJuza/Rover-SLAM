import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_bringup = get_package_share_directory('rover_bringup')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_frontier = get_package_share_directory('frontier_exploration_ros2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    frontier_params_file = LaunchConfiguration('frontier_params_file')

    # 1. Nav2 Navigation Stack in SLAM mode (Composition enabled on Pi, AMCL/map_server disabled)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_params_file,
            'use_composition': 'False',
        }.items(),
    )

    # 2. Frontier Explorer Node (frontier_exploration_ros2)
    frontier_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_frontier, 'launch', 'frontier_explorer.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': frontier_params_file,
            'autostart': 'true',
            'map_qos_durability': 'transient_local',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(pkg_bringup, 'config', 'nav2_slam_params.yaml'),
            description='Nav2 configuration file for SLAM exploration'
        ),
        DeclareLaunchArgument(
            'frontier_params_file',
            default_value=os.path.join(pkg_bringup, 'config', 'frontier_params.yaml'),
            description='Frontier exploration parameters file'
        ),
        nav2_launch,
        frontier_launch,
    ])
