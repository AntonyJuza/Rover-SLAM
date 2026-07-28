import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_rover = get_package_share_directory('rover_bringup')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    default_map = '/home/juza/rover-rpi/my_map.yaml'

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load'
    )

    # 1. Gazebo Sim Launch (with SLAM disabled so Nav2 map_server & AMCL can run)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover, 'launch', 'gazebo_sim.launch.py')
        ),
        launch_arguments={
            'use_slam': 'False',
            'use_sim_time': 'true'
        }.items()
    )

    # 2. Nav2 Bringup Launch (Map Server + AMCL + Costmaps + BT Navigator)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
            'slam': 'False',
            'use_composition': 'False',
            'params_file': os.path.join(pkg_rover, 'config', 'nav2_params.yaml')
        }.items()
    )

    return LaunchDescription([
        map_arg,
        gazebo_sim,
        nav2_bringup
    ])
