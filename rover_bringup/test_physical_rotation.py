#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
import math
import time


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RotationTestNode(Node):
    def __init__(self):
        super().__init__('rotation_test_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Rotation Test Node initialized. Ready for $360^\\circ$ turn test.')

    def get_current_yaw(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('odom', 'base_footprint', now, rclpy.duration.Duration(seconds=1.0))
            yaw = quaternion_to_yaw(trans.transform.rotation)
            return yaw
        except Exception as e:
            return None

    def run_test(self, speed=0.5, duration=5.0):
        self.get_logger().info('Waiting for TF transform...')
        time.sleep(2.0)
        
        start_yaw = self.get_current_yaw()
        while start_yaw is None:
            time.sleep(0.5)
            start_yaw = self.get_current_yaw()

        self.get_logger().info(f'--- ROTATION TEST START ---')
        self.get_logger().info(f'Starting TF Yaw: {math.degrees(start_yaw):.2f}°')
        self.get_logger().info(f'Rotating robot at {speed} rad/s for {duration} seconds...')

        twist = Twist()
        twist.angular.z = speed
        
        start_time = time.time()
        accumulated_yaw = 0.0
        last_yaw = start_yaw

        while (time.time() - start_time) < duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
            
            curr_yaw = self.get_current_yaw()
            if curr_yaw is not None:
                dy = curr_yaw - last_yaw
                # Normalize angle diff (-pi to pi)
                while dy > math.pi:
                    dy -= 2 * math.pi
                while dy < -math.pi:
                    dy += 2 * math.pi
                accumulated_yaw += dy
                last_yaw = curr_yaw

        # Stop robot
        stop_twist = Twist()
        self.cmd_pub.publish(stop_twist)
        time.sleep(1.0)

        end_yaw = self.get_current_yaw()
        self.get_logger().info(f'--- ROTATION TEST COMPLETED ---')
        self.get_logger().info(f'End TF Yaw: {math.degrees(end_yaw):.2f}°')
        self.get_logger().info(f'Total Reported TF Rotation: {math.degrees(accumulated_yaw):.2f}°')


def main(args=None):
    rclpy.init(args=args)
    node = RotationTestNode()
    try:
        node.run_test(speed=0.5, duration=6.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
