#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener
import math
import time


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw_rad)


class RotationComparator(Node):
    def __init__(self):
        super().__init__('rotation_comparator')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_odom_yaw = None
        self.latest_imu_yaw = None

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

        self.timer = self.create_timer(0.2, self.timer_cb)

        self.get_logger().info('Rotation Comparator initialized. Printing angle comparisons...')
        print("\n" + "=" * 75)
        print(f"{'TIMESTAMP':<10} | {'/odom YAW':<12} | {'/imu YAW':<12} | {'/tf (RViz) YAW':<14} | {'DIFF (TF - ODOM)':<16}")
        print("=" * 75)

    def odom_cb(self, msg):
        self.latest_odom_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def imu_cb(self, msg):
        self.latest_imu_yaw = quaternion_to_yaw(msg.orientation)

    def timer_cb(self):
        tf_yaw = None
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            tf_yaw = quaternion_to_yaw(t.transform.rotation)
        except Exception:
            pass

        now_str = f"{time.strftime('%H:%M:%S')}"
        odom_str = f"{self.latest_odom_yaw:+.2f}°" if self.latest_odom_yaw is not None else "N/A"
        imu_str = f"{self.latest_imu_yaw:+.2f}°" if self.latest_imu_yaw is not None else "N/A"
        tf_str = f"{tf_yaw:+.2f}°" if tf_yaw is not None else "N/A"

        diff_str = "N/A"
        if tf_yaw is not None and self.latest_odom_yaw is not None:
            diff = tf_yaw - self.latest_odom_yaw
            diff_str = f"{diff:+.2f}°"

        print(f"{now_str:<10} | {odom_str:<12} | {imu_str:<12} | {tf_str:<14} | {diff_str:<16}")


def main(args=None):
    rclpy.init(args=args)
    node = RotationComparator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
