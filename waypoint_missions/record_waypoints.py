#!/usr/bin/env python3
"""
Rover Waypoint Recorder
=======================
Records live waypoints directly from the rover's current pose in the map frame (via TF)
or from RViz clicks, and saves them to waypoints.yaml.

Usage:
  1. Interactive terminal recorder (Drive rover, then name & save each spot):
     python3 record_waypoints.py --interactive

  2. Save current pose as a named waypoint:
     python3 record_waypoints.py --name "charging_dock" --wait 5.0 --action wait

  3. Listen to RViz 2D Goal Pose clicks and save each click:
     python3 record_waypoints.py --from-rviz
"""

import argparse
import math
import os
import sys
import yaml

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
import tf2_ros


def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion (x, y, z, w) to yaw in radians."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointRecorder(Node):
    def __init__(self, yaml_path):
        super().__init__('waypoint_recorder')
        self.yaml_path = yaml_path
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.waypoints = self.load_existing_waypoints()

    def load_existing_waypoints(self):
        if os.path.exists(self.yaml_path):
            try:
                with open(self.yaml_path, 'r') as f:
                    data = yaml.safe_load(f)
                if data and 'waypoints' in data and isinstance(data['waypoints'], list):
                    return data['waypoints']
            except Exception as e:
                self.get_logger().warn(f"Could not load existing file: {e}")
        return []

    def save_waypoints(self):
        os.makedirs(os.path.dirname(os.path.abspath(self.yaml_path)), exist_ok=True)
        data = {'waypoints': self.waypoints}
        with open(self.yaml_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        self.get_logger().info(f"Saved {len(self.waypoints)} waypoints to {self.yaml_path}")

    def get_current_rover_pose(self, timeout_sec=3.0):
        """Query TF for map -> base_footprint transformation."""
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                trans = self.tf_buffer.lookup_transform(
                    'map',
                    'base_footprint',
                    Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                x = trans.transform.translation.x
                y = trans.transform.translation.y
                qx = trans.transform.rotation.x
                qy = trans.transform.rotation.y
                qz = trans.transform.rotation.z
                qw = trans.transform.rotation.w
                yaw = quaternion_to_yaw(qx, qy, qz, qw)
                return x, y, yaw
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
        return None

    def add_waypoint(self, name, x, y, yaw, wait_sec=3.0, action='wait'):
        entry = {
            'name': name,
            'x': round(float(x), 3),
            'y': round(float(y), 3),
            'yaw': round(float(yaw), 3),
            'wait_seconds': float(wait_sec),
            'action': str(action)
        }
        self.waypoints.append(entry)
        self.save_waypoints()
        print(f"--> [SAVED] '{name}': x={entry['x']}, y={entry['y']}, yaw={entry['yaw']} rad ({math.degrees(yaw):.1f}°)")


def rviz_mode(recorder):
    """Save waypoints whenever a 2D Nav Goal is clicked in RViz."""
    print("\n[RVIZ MODE] Listening for '2D Goal Pose' clicks in RViz on topic /goal_pose...")
    print("Press Ctrl+C to stop recording.\n")

    count = len(recorder.waypoints)

    def goal_callback(msg: PoseStamped):
        nonlocal count
        count += 1
        x = msg.pose.position.x
        y = msg.pose.position.y
        o = msg.pose.orientation
        yaw = quaternion_to_yaw(o.x, o.y, o.z, o.w)
        name = f"Point_{count}"
        recorder.add_waypoint(name, x, y, yaw, wait_sec=3.0, action='wait')

    recorder.create_subscription(PoseStamped, '/goal_pose', goal_callback, 10)
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        print("\nFinished recording RViz waypoints.")


def interactive_mode(recorder):
    """Drive rover to locations, name each spot, and save to YAML."""
    print("\n=======================================================")
    print("           INTERACTIVE WAYPOINT RECORDER")
    print("=======================================================")
    print("Instructions:")
    print("1. Drive the rover to your desired waypoint (via teleop or RViz).")
    print("2. Enter a name for the waypoint and press Enter.")
    print("3. Type 'q' or 'exit' when finished.\n")

    wp_count = len(recorder.waypoints)

    while rclpy.ok():
        default_name = f"Point_{wp_count + 1}"
        try:
            user_input = input(f"Enter waypoint name [{default_name}] (or 'q' to quit): ").strip()
        except (KeyboardInterrupt, EOFError):
            break

        if user_input.lower() in ['q', 'exit']:
            break

        name = user_input if user_input else default_name
        print(f"Reading current rover pose from TF (map -> base_footprint)...")
        pose = recorder.get_current_rover_pose()

        if pose is None:
            print("[ERROR] Could not read transform 'map' -> 'base_footprint'. Ensure Nav2 / AMCL / TF is running!")
            continue

        x, y, yaw = pose
        recorder.add_waypoint(name, x, y, yaw, wait_sec=3.0, action='wait')
        wp_count += 1

    print(f"\n[INFO] Interactive recording finished. Total waypoints in file: {len(recorder.waypoints)}")


def main():
    parser = argparse.ArgumentParser(description="Rover Waypoint Recorder")
    parser.add_argument('--file', '-f', default=os.path.join(os.path.dirname(__file__), 'waypoints.yaml'),
                        help="Path to save waypoints.yaml")
    parser.add_argument('--name', '-n', help="Name of waypoint to save immediately from current pose")
    parser.add_argument('--wait', '-w', type=float, default=3.0, help="Wait duration in seconds (default 3.0)")
    parser.add_argument('--action', '-a', choices=['wait', 'photo', 'none'], default='wait',
                        help="Action at waypoint (default 'wait')")
    parser.add_argument('--interactive', '-i', action='store_true',
                        help="Run interactive mode to name and save multiple spots")
    parser.add_argument('--from-rviz', action='store_true',
                        help="Record waypoints by clicking 2D Goal Pose in RViz")

    args = parser.parse_args()

    rclpy.init()
    recorder = WaypointRecorder(args.file)

    try:
        if args.from_rviz:
            rviz_mode(recorder)
        elif args.interactive:
            interactive_mode(recorder)
        elif args.name:
            pose = recorder.get_current_rover_pose()
            if pose is None:
                print("[ERROR] Could not read robot pose from TF. Make sure Nav2/AMCL is active.")
                sys.exit(1)
            x, y, yaw = pose
            recorder.add_waypoint(args.name, x, y, yaw, wait_sec=args.wait, action=args.action)
        else:
            interactive_mode(recorder)
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
