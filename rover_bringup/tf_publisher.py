import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')
        self.broadcaster = TransformBroadcaster(self)
        # Publish at 50Hz with real timestamps
        self.timer = self.create_timer(0.02, self.publish_transforms)
        self.get_logger().info('TF publisher started')

    def publish_transforms(self):
        now = self.get_clock().now().to_msg()

        transforms = []

        # base_link → laser
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'base_link'
        t1.child_frame_id = 'laser'
        t1.transform.translation.x = -0.08
        t1.transform.translation.y =  0.0
        t1.transform.translation.z =  0.08
        t1.transform.rotation.w = 1.0
        transforms.append(t1)

        # base_link → imu_link
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'imu_link'
        t2.transform.translation.x =  0.02
        t2.transform.translation.y = -0.04
        t2.transform.translation.z =  0.047
        t2.transform.rotation.w = 1.0
        transforms.append(t2)

        self.broadcaster.sendTransform(transforms)


def main():
    rclpy.init()
    node = StaticTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
