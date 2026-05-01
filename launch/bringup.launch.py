import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg         = get_package_share_directory('rover_bringup')
    pkg_motor   = get_package_share_directory('humanoid_motor_control')
    pkg_bno055  = get_package_share_directory('bno055_ser')


    # ── Process URDF ──────────────────────────────────────────────────────────
    # Using BNO055.urdf.xacro — no Pixhawk in this branch
    xacro_file = os.path.join(pkg, 'urdf', 'BNO055.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # =========================================================================
    # 1. robot_state_publisher
    #    Reads URDF → publishes static TFs:
    #      base_footprint→base_link, base_link→laser,
    #      base_link→imu_link, base_link→wheels
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
    # 2. BNO055 IMU Driver (replaces MAVROS/Pixhawk in this branch)
    #    Reads BNO055 over I2C → publishes /imu/bno055 (sensor_msgs/Imu)
    #    Publishes: orientation (quaternion) + angular velocity + linear accel
    # =========================================================================
    bno055_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bno055, 'launch', 'bno055_launch.py')
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
            'frame_id':  'laser_frame',
            'skip':      2,    # 10Hz (skip every other scan)
        }],
        output='screen',
    )

    # =========================================================================
    # 5. EKF — robot_localization
    #    Fuses: /odom (encoders) + /imu/bno055 (BNO055 IMU)
    #    Publishes: /odometry/filtered  +  odom→base_footprint TF
    # =========================================================================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(pkg, 'config', 'ekf.yaml')],
        output='screen',
        emulate_tty=True,
    )


    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,

        bno055_launch,
        motor_launch,
        lidar_node,
        ekf_node,
    ])
