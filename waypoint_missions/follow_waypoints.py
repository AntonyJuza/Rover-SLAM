#!/usr/bin/env python3
"""
Rover Waypoint Mission Runner
=============================
Navigates the rover through a sequence of waypoints defined in a YAML file using Nav2.
Supports:
  - Sequential waypoint navigation with custom action hooks (wait, take photo, sensor capture)
  - Native Nav2 FollowWaypoints mode
  - Patrol looping (--loop N or --loop -1 for infinite)
  - Return-to-home option
  - Graceful cancellation on Ctrl+C

Usage:
  python3 follow_waypoints.py
  python3 follow_waypoints.py --file waypoints.yaml --mode sequential
  python3 follow_waypoints.py --mode nav2
  python3 follow_waypoints.py --loop 3 --return-home
"""

import argparse
import math
import os
import sys
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def euler_to_quaternion(yaw, pitch=0.0, roll=0.0):
    """Convert yaw (rad) to quaternion (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def load_waypoints(yaml_path):
    """Load and validate waypoints from a YAML file."""
    if not os.path.exists(yaml_path):
        print(f"[ERROR] Waypoints file not found: {yaml_path}")
        sys.exit(1)

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    if not data or 'waypoints' not in data:
        print(f"[ERROR] Invalid format in {yaml_path}. 'waypoints' list required.")
        sys.exit(1)

    waypoints = data['waypoints']
    print(f"[INFO] Loaded {len(waypoints)} waypoint(s) from {yaml_path}")
    for idx, wp in enumerate(waypoints):
        name = wp.get('name', f'WP_{idx}')
        x = wp.get('x', 0.0)
        y = wp.get('y', 0.0)
        yaw = wp.get('yaw', 0.0)
        action = wp.get('action', 'none')
        wait_sec = wp.get('wait_seconds', 0.0)
        print(f"  [{idx + 1}] {name}: (x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad) | action={action} (wait={wait_sec}s)")
    return waypoints


def create_pose_stamped(navigator, x, y, yaw, frame_id='map'):
    """Create a PoseStamped message from 2D coordinates and yaw."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    qx, qy, qz, qw = euler_to_quaternion(float(yaw))
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def execute_waypoint_task(wp, camera_topic='/camera/image_raw'):
    """
    Execute task hook when arriving at a waypoint.
    Can be expanded to capture camera frames, read sensors, operate an arm, etc.
    """
    name = wp.get('name', 'Waypoint')
    action = wp.get('action', 'wait').lower()
    wait_sec = float(wp.get('wait_seconds', 2.0))

    print(f"\n---> [TASK] Arrived at '{name}'. Executing action: '{action}'")

    if action == 'photo':
        print(f"     [CAMERA] Requesting image capture for waypoint '{name}'...")
        # Placeholder / hook for camera snapshot
        # You can subscribe to camera_topic or use cv_bridge here
        os.makedirs("photos", exist_ok=True)
        photo_filename = f"photos/{name}_{int(time.time())}.txt"
        with open(photo_filename, "w") as pf:
            pf.write(f"Snapshot taken at waypoint '{name}' at {time.ctime()}\n")
        print(f"     [CAMERA] Snapshot metadata logged to {photo_filename}")
        if wait_sec > 0:
            print(f"     [WAIT] Holding position for {wait_sec:.1f}s...")
            time.sleep(wait_sec)

    elif action == 'wait':
        print(f"     [WAIT] Pausing at '{name}' for {wait_sec:.1f} seconds...")
        for remaining in range(int(wait_sec), 0, -1):
            print(f"     ... {remaining}s remaining", end='\r', flush=True)
            time.sleep(1.0)
        time.sleep(wait_sec - int(wait_sec))  # fractional part
        print(f"     [WAIT] Pause completed.")

    elif action == 'none':
        print("     [NONE] Continuing directly to next destination.")

    else:
        print(f"     [CUSTOM] Performing custom task '{action}'...")
        if wait_sec > 0:
            time.sleep(wait_sec)

    print(f"---> [TASK] Task at '{name}' finished.\n")


def run_sequential_mission(navigator, waypoints, camera_topic='/camera/image_raw'):
    """Visit each waypoint sequentially using goToPose and trigger task hooks."""
    total_wps = len(waypoints)

    for idx, wp in enumerate(waypoints):
        name = wp.get('name', f'WP_{idx + 1}')
        x = wp.get('x', 0.0)
        y = wp.get('y', 0.0)
        yaw = wp.get('yaw', 0.0)

        print(f"\n=======================================================")
        print(f"[MISSION] Navigating to Waypoint [{idx + 1}/{total_wps}]: {name}")
        print(f"          Target: (x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad / {math.degrees(yaw):.1f}°)")
        print(f"=======================================================")

        target_pose = create_pose_stamped(navigator, x, y, yaw)
        navigator.goToPose(target_pose)

        loop_count = 0
        while not navigator.isTaskComplete():
            loop_count += 1
            feedback = navigator.getFeedback()
            if feedback and loop_count % 5 == 0:
                dist = feedback.distance_remaining
                print(f"  [Progress] Distance to '{name}': {dist:.2f} m", end='\r', flush=True)
            time.sleep(0.2)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"\n[SUCCESS] Reached waypoint [{idx + 1}/{total_wps}]: {name}")
            execute_waypoint_task(wp, camera_topic)
        elif result == TaskResult.CANCELED:
            print(f"\n[CANCEL] Navigation to '{name}' was canceled.")
            return False
        elif result == TaskResult.FAILED:
            print(f"\n[FAILURE] Failed to reach '{name}'. Proceeding to next if possible.")
        else:
            print(f"\n[UNKNOWN] Unknown task result: {result}")

    return True


def run_nav2_follow_waypoints(navigator, waypoints):
    """Run native Nav2 followWaypoints action."""
    print("\n[MISSION] Preparing Nav2 FollowWaypoints batch...")
    poses = []
    for wp in waypoints:
        poses.append(create_pose_stamped(navigator, wp['x'], wp['y'], wp['yaw']))

    navigator.followWaypoints(poses)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f"  [Nav2 FollowWaypoints] At waypoint index: {feedback.current_waypoint + 1}/{len(poses)}", end='\r', flush=True)
        time.sleep(0.2)

    result = navigator.getResult()
    print("")
    if result == TaskResult.SUCCEEDED:
        print("[SUCCESS] All waypoints visited successfully!")
        return True
    elif result == TaskResult.CANCELED:
        print("[CANCEL] Waypoint following was canceled.")
        return False
    else:
        print(f"[FAILURE] Nav2 followWaypoints ended with status: {result}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Rover Nav2 Waypoint Missions")
    parser.add_argument('--file', '-f', default=os.path.join(os.path.dirname(__file__), 'waypoints.yaml'),
                        help="Path to waypoints.yaml file")
    parser.add_argument('--mode', '-m', choices=['sequential', 'nav2'], default='sequential',
                        help="Navigation mode: 'sequential' (with task hooks) or 'nav2' (batch followWaypoints)")
    parser.add_argument('--loop', '-l', type=int, default=1,
                        help="Number of times to loop the mission (use -1 for infinite patrol)")
    parser.add_argument('--return-home', '-r', action='store_true',
                        help="Return to first waypoint (Home) at the end of the mission")
    parser.add_argument('--camera-topic', default='/camera/image_raw',
                        help="Camera ROS topic for photo actions")

    args = parser.parse_args()

    # Load waypoints
    waypoints = load_waypoints(args.file)
    if not waypoints:
        print("[ERROR] No waypoints found to execute.")
        sys.exit(1)

    # Initialize ROS 2 & BasicNavigator
    rclpy.init()
    navigator = BasicNavigator()

    print("[INFO] Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()
    print("[INFO] Nav2 is active and ready!")

    loop_count = 0
    max_loops = args.loop

    try:
        while True:
            loop_count += 1
            if max_loops > 0:
                print(f"\n=======================================================")
                print(f"       STARTING MISSION CYCLE {loop_count} OF {max_loops}")
                print(f"=======================================================")
            else:
                print(f"\n=======================================================")
                print(f"       PATROL CYCLE {loop_count} (INFINITE MODE)")
                print(f"=======================================================")

            if args.mode == 'sequential':
                success = run_sequential_mission(navigator, waypoints, args.camera_topic)
            else:
                success = run_nav2_follow_waypoints(navigator, waypoints)

            if not success:
                print("[WARNING] Mission cycle aborted early.")
                break

            if max_loops > 0 and loop_count >= max_loops:
                break

            print("[INFO] Cycle finished. Brief pause before next cycle...")
            time.sleep(2.0)

        # Optional Return to Home
        if args.return_home and waypoints:
            print("\n=======================================================")
            print("[MISSION] Returning to Home position...")
            print("=======================================================")
            home_wp = waypoints[0]
            home_pose = create_pose_stamped(navigator, home_wp['x'], home_wp['y'], home_wp['yaw'])
            navigator.goToPose(home_pose)
            while not navigator.isTaskComplete():
                time.sleep(0.2)
            print("[SUCCESS] Rover is back at Home!")

    except KeyboardInterrupt:
        print("\n\n[WARNING] Mission interrupted by user (Ctrl+C). Canceling Nav2 tasks...")
        navigator.cancelTask()
        print("[INFO] Rover stopped safely.")

    finally:
        rclpy.shutdown()
        print("[INFO] Mission script exited.")


if __name__ == '__main__':
    main()
