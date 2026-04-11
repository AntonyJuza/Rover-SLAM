import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg         = get_package_share_directory('rover_bringup')
    pkg_motor   = get_package_share_directory('humanoid_motor_control')
    imu_pkg     = get_package_share_directory('ros2_mpu6050')

    # ── Process URDF ──────────────────────────────────────────────────────────
    xacro_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # =========================================================================
    # 1. robot_state_publisher
    #    Reads URDF → publishes static TFs:
    #      base_footprint→base_link, base_link→laser,
    #      base_link→imu_link, base_link→wheels
    #    Replaces the manual static_transform_publisher you were running.
    # =========================================================================
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



    # =========================================================================
    # 2. MPU6050 IMU
    #    Was: ros2 launch ros2_mpu6050 ros2_mpu6050.launch.py
    #    Publishes: /imu/mpu6050
    # =========================================================================
    imu_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_pkg, 'launch', 'ros2_mpu6050.launch.py')
        ),
    )

    # =========================================================================
    # 3. Motor control launch (includes cytron + encoder + diff_drive)
    #    Was: ros2 launch humanoid_motor_control motor_control.launch.py
    #    Publishes: /odom  (publish_tf=false — EKF owns odom→base_link)
    #
    #    IMPORTANT: Copy rover_bringup/config/motor_control.yaml to:
    #    ~/ros2_jazzy/src/humanoid_motor_control/config/motor_control.yaml
    #    (publish_tf changed to false, wheel params corrected)
    # =========================================================================
    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_motor, 'launch', 'motor_control.launch.py')
        ),
    )

    # =========================================================================
    # 4. Lidar driver
    #    Was: ros2 run sdkeli_ls_udp sdkeli_ls1207de --ros-args -p hostname:=...
    #    Publishes: /scan  (frame_id: laser — matches URDF)
    # =========================================================================
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

    # =========================================================================
    # 5. EKF — robot_localization
    #    Was: ros2 run robot_localization ekf_node --ros-args --params-file ekf.yaml
    #    Fuses: /odom (encoders) + /imu/mpu6050
    #    Publishes: /odometry/filtered  +  odom→base_link TF
    # =========================================================================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(pkg, 'config', 'ekf.yaml')],
        output='screen',
        emulate_tty=True,
    )

    # =========================================================================
    # 6. SLAM Toolbox
    #    Was: ros2 launch slam_toolbox online_async_launch.py params_file:=...
    #    Subscribes: /scan + full TF chain
    #    Publishes:  /map  +  map→odom TF
    # =========================================================================
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
        robot_state_publisher,
        joint_state_publisher,
        imu_node,
        motor_launch,
        lidar_node,
        ekf_node,
        slam_node,
    ])
