import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.executors import ExternalShutdownException


class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)

        # Safely declare parameters
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        if not self.has_parameter('use_imu'):
            self.declare_parameter('use_imu', False)

        self.use_imu = bool(self.get_parameter('use_imu').value)
        self.latest_imu_orientation = None
        self.latest_imu_yaw = 0.0

        # Dead-reckoning position state when using IMU
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.last_stamp = None

        # Subscribe to /odom (Wheel Encoders for Velocity)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        if self.use_imu:
            self.imu_sub = self.create_subscription(
                Imu,
                '/imu/data',
                self.imu_callback,
                10
            )
            self.get_logger().info('Odom TF Publisher running in IMU DEAD-RECKONING MODE (V_x integrated with IMU Heading).')
        else:
            self.get_logger().info('Odom TF Publisher running in PURE ENCODER MODE.')

    def imu_callback(self, msg: Imu):
        self.latest_imu_orientation = msg.orientation
        # Extract yaw angle from quaternion
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.latest_imu_yaw = math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'odom'
        t.child_frame_id = msg.child_frame_id if msg.child_frame_id else 'base_footprint'

        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.use_imu and self.latest_imu_orientation is not None:
            if self.last_stamp is not None:
                dt = current_time - self.last_stamp
                if 0.0 < dt < 1.0:
                    vx = msg.twist.twist.linear.x
                    self.pos_x += vx * math.cos(self.latest_imu_yaw) * dt
                    self.pos_y += vx * math.sin(self.latest_imu_yaw) * dt
            self.last_stamp = current_time

            t.transform.translation.x = self.pos_x
            t.transform.translation.y = self.pos_y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = self.latest_imu_orientation
        else:
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation

        # Broadcast transform to ROS 2 /tf
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
