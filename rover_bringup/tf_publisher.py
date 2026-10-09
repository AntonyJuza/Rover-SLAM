import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


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

        # Subscribe to /odom (Wheel Encoders for Position)
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
            self.get_logger().info('Odom TF Publisher running in HYBRID MODE (Encoders for Position, BNO055 IMU for Heading).')
        else:
            self.get_logger().info('Odom TF Publisher running in PURE ENCODER MODE.')

    def imu_callback(self, msg: Imu):
        self.latest_imu_orientation = msg.orientation

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'odom'
        t.child_frame_id = msg.child_frame_id if msg.child_frame_id else 'base_footprint'

        # Position 100% from Wheel Encoders
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Orientation: BNO055 IMU if enabled and received, otherwise Wheel Encoders
        if self.use_imu and self.latest_imu_orientation is not None:
            t.transform.rotation = self.latest_imu_orientation
        else:
            t.transform.rotation = msg.pose.pose.orientation

        # Broadcast transform to ROS 2 /tf
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
