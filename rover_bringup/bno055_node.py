#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import serial
import math
import time


def quaternion_to_yaw(w, x, y, z):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class BNO055SerialNode(Node):
    def __init__(self):
        super().__init__('bno055_serial_node')
        
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        
        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)

        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.yaw_pub = self.create_publisher(Float64, '/imu/yaw', 10)

        self.get_logger().info(f'Connecting to BNO055 Arduino Nano on {self.port} at {self.baud} baud...')

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
            time.sleep(1.0)
            self.get_logger().info('Serial port opened successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {self.port}: {e}')
            self.ser = None

        self.timer = self.create_timer(0.01, self.read_serial)  # 100Hz read loop

    def read_serial(self):
        if self.ser is None or not self.ser.is_open:
            return

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('QUAT:'):
                    parts = line.replace('QUAT:', '').split(',')
                    if len(parts) == 4:
                        w = float(parts[0])
                        x = float(parts[1])
                        y = float(parts[2])
                        z = float(parts[3])

                        # Create IMU message
                        imu_msg = Imu()
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = 'imu_link'
                        imu_msg.orientation.w = w
                        imu_msg.orientation.x = x
                        imu_msg.orientation.y = y
                        imu_msg.orientation.z = z

                        self.imu_pub.publish(imu_msg)

                        # Publish Yaw angle (radians)
                        yaw = quaternion_to_yaw(w, x, y, z)
                        yaw_msg = Float64()
                        yaw_msg.data = yaw
                        self.yaw_pub.publish(yaw_msg)

        except Exception as e:
            self.get_logger().warn(f'Error reading serial: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BNO055SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
