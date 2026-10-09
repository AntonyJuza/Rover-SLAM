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
    pkg_bringup = get_package_share_directory('rover_bringup')
    pkg_desc    = get_package_share_directory('rover_description')
    pkg_motor   = get_package_share_directory('humanoid_motor_control')

    use_imu    = LaunchConfiguration('use_imu')
    use_slam   = LaunchConfiguration('use_slam')
    left_trim  = LaunchConfiguration('left_trim')
    right_trim = LaunchConfiguration('right_trim')
    trim       = LaunchConfiguration('trim')

    # 1. Process Robot Description (URDF / CAD model from rover_description)
    xacro_file = os.path.join(pkg_desc, 'urdf', 'rover.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
            'publish_frequency': 20.0,
        }],
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    # 2. BNO055 XIAO IMU Driver (Seeed XIAO RA4M1 on USB /dev/ttyACM0)
    bno055_node = Node(
        package='bno055_xiao_driver',
        executable='bno055_xiao_node',
        name='bno055_xiao_node',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'frame_id': 'imu_link',
            'publish_tf': False,
        }],
        condition=IfCondition(use_imu),
        output='screen'
    )

    # 3. SDKELI LS1207DE LiDAR Driver (UDP 192.168.64.100:2112)
    lidar_node = Node(
        package='sdkeli_lidar',
        executable='sdkeli_ls1207de',
        name='sdkeli_ls1207de',
        parameters=[{
            'hostname': '192.168.64.100',
            'port': '2112',
            'frame_id': 'laser',
            'range_max': 20.0,
            'time_increment': 0.000040,
        }],
        output='screen'
    )

    # 4. Motor Control Launch (Cytron MDDRC10 + SCX3530 Encoders + Diff Drive)
    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_motor, 'launch', 'motor_control.launch.py')
        ),
        launch_arguments={
            'left_trim': left_trim,
            'right_trim': right_trim,
            'trim': trim,
        }.items(),
    )

    # 5. Odom TF Publisher (Fuses /odom wheel position + /imu/data yaw into odom->base_footprint)
    odom_tf_publisher = Node(
        package='rover_bringup',
        executable='tf_publisher',
        name='odom_tf_publisher',
        parameters=[{
            'use_sim_time': False,
            'use_imu': use_imu,
        }],
        output='screen'
    )

    # 6. SLAM Toolbox (Online Async SLAM)
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_bringup, 'config', 'slam_params.yaml'),
            'use_sim_time': 'false'
        }.items(),
        condition=IfCondition(use_slam)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_imu',
            default_value='true',
            description='Enable BNO055 IMU orientation fusion'
        ),
        DeclareLaunchArgument(
            'use_slam',
            default_value='true',
            description='Start SLAM Toolbox node'
        ),
        DeclareLaunchArgument(
            'left_trim',
            default_value='1.0',
            description='Multiplier for left motor (default: 1.0)'
        ),
        DeclareLaunchArgument(
            'right_trim',
            default_value='0.85',
            description='Multiplier for right motor (default: 0.85)'
        ),
        DeclareLaunchArgument(
            'trim',
            default_value='0.0',
            description='Differential steering trim (-1.0 to 1.0)'
        ),
        robot_state_publisher,
        joint_state_publisher,
        bno055_node,
        lidar_node,
        motor_launch,
        odom_tf_publisher,
        slam_node,
    ])
