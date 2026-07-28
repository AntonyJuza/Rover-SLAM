import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg         = get_package_share_directory('rover_bringup')
    pkg_motor   = get_package_share_directory('humanoid_motor_control')

    use_imu = LaunchConfiguration('use_imu', default='false')

    # ── Process URDF ──────────────────────────────────────────────────────────
    xacro_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # 1. robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
            'publish_frequency': 15.0,
            'ignore_timestamp': True, 
        }],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}],
    )

    # 2. Motor control launch (includes cytron + encoder + diff_drive)
    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_motor, 'launch', 'motor_control.launch.py')
        ),
    )

    # 3. Lidar driver
    lidar_node = Node(
        package='sdkeli_ls_udp',
        executable='sdkeli_ls1207de',
        name='sdkeli_ls1207de',
        parameters=[{
            'hostname':  '192.168.64.100',
            'port':      2112,
            'range_max': 200.0,
            'frame_id':  'laser',
        }],
        output='screen',
    )

    # 4. BNO055 Serial Node (Arduino Nano)
    bno055_node = Node(
        package='rover_bringup',
        executable='bno055_node',
        name='bno055_serial_node',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'baud': 115200,
        }],
        condition=IfCondition(use_imu),
        output='screen',
    )

    # 5. Odom TF Publisher
    odom_tf_publisher = Node(
        package='rover_bringup',
        executable='tf_publisher',
        name='odom_tf_publisher',
        parameters=[{
            'use_sim_time': False,
            'use_imu': use_imu,
        }],
        output='screen',
    )

    # 6. SLAM Toolbox
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(pkg, 'config', 'slam_params.yaml'),
            'use_sim_time': 'false'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_imu',
            default_value='false',
            description='Enable BNO055 IMU orientation fusion'
        ),
        robot_state_publisher,
        joint_state_publisher,
        motor_launch,
        lidar_node,
        bno055_node,
        odom_tf_publisher,
        slam_node,
    ])
