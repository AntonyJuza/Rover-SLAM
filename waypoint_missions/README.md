# Rover Waypoint Missions Guide

This package provides a complete waypoint navigation and mission execution system for the rover using **Nav2 (ROS 2 Jazzy)**.

---

## 📁 Files Included

| File | Description |
| :--- | :--- |
| [`waypoints.yaml`](file:///home/juza/rover_slam_ws/waypoint_missions/waypoints.yaml) | YAML database storing named locations `(x, y, yaw)` and actions (`wait`, `photo`, `none`). |
| [`follow_waypoints.py`](file:///home/juza/rover_slam_ws/waypoint_missions/follow_waypoints.py) | Mission executor supporting sequential task hooks, patrol loops, and native Nav2 batching. |
| [`record_waypoints.py`](file:///home/juza/rover_slam_ws/waypoint_missions/record_waypoints.py) | Interactive CLI tool to capture live coordinates directly from the robot in RViz or TF. |

---

## 📍 Step 1: Recording Waypoints

Instead of guessing coordinates from the map, you can record them live while driving or clicking in RViz:

### Option A: Interactive Drive & Record (Recommended)
1. In a terminal on the workstation, launch teleop or use RViz to drive the rover to a spot:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_smoothed
   ```
2. In another terminal, run the recorder:
   ```bash
   python3 /home/juza/rover_slam_ws/waypoint_missions/record_waypoints.py --interactive
   ```
3. Type the waypoint name (e.g., `Charging_Dock`, `Kitchen`, `Doorway`) and press **Enter**. The script queries `map -> base_footprint` TF and appends the pose to `waypoints.yaml`.

### Option B: Click & Save via RViz
Run the recorder in RViz listener mode:
```bash
python3 /home/juza/rover_slam_ws/waypoint_missions/record_waypoints.py --from-rviz
```
Then in RViz, click the **2D Goal Pose** tool on the map. Every click is automatically captured and saved as a sequential waypoint.

---

## 🚀 Step 2: Running a Waypoint Mission

Make sure Nav2 bringup is running on the rover / workstation, then run:

### 1. Sequential Mode with Tasks (Default)
Visits each waypoint, displays distance feedback, and executes each waypoint's custom action (`wait`, `photo`):
```bash
python3 /home/juza/rover_slam_ws/waypoint_missions/follow_waypoints.py
```

### 2. Patrol / Repeating Loop
Repeats the route multiple times (or infinite patrol) and returns to Home at the end:
```bash
# Repeat 3 times, then return to Home:
python3 /home/juza/rover_slam_ws/waypoint_missions/follow_waypoints.py --loop 3 --return-home

# Infinite patrol loop:
python3 /home/juza/rover_slam_ws/waypoint_missions/follow_waypoints.py --loop -1
```

### 3. Native Nav2 Batch Mode
Sends all points in one batch to Nav2's native `FollowWaypoints` action server:
```bash
python3 /home/juza/rover_slam_ws/waypoint_missions/follow_waypoints.py --mode nav2
```

---

## 🛠️ Customizing Waypoint Actions

In [`waypoints.yaml`](file:///home/juza/rover_slam_ws/waypoint_missions/waypoints.yaml), each point can have its own behavior:

```yaml
waypoints:
  - name: "Doorway"
    x: 1.25
    y: -0.42
    yaw: 1.57        # Facing North (rad)
    wait_seconds: 5.0 # Pause 5 seconds
    action: "wait"

  - name: "Inspection_Point"
    x: 2.10
    y: 1.05
    yaw: 0.0
    wait_seconds: 3.0
    action: "photo"   # Triggers camera snapshot
```

### Adding New Custom Tasks (Camera, Arm, Sensors)
Open [`follow_waypoints.py`](file:///home/juza/rover_slam_ws/waypoint_missions/follow_waypoints.py) and locate the `execute_waypoint_task(wp)` function:
```python
def execute_waypoint_task(wp, camera_topic='/camera/image_raw'):
    action = wp.get('action', 'wait').lower()
    
    if action == 'photo':
        # Hook: Call camera service, capture frame from OpenCV or ROS topic
        ...
    elif action == 'arm_pick':
        # Hook: Send goal to robotic arm action server
        ...
```

---

## 🎯 Visual Waypoint Mode in RViz (Zero-Code)

Nav2 also supports clicking waypoints directly in RViz:
1. Open the Nav2 panel in RViz (left or bottom dock).
2. Change the Navigation Mode dropdown from **Navigate to Pose** to **Waypoint / Through Poses mode**.
3. Click the **Nav2 Goal** tool on the map to place Waypoint 1, Waypoint 2, Waypoint 3...
4. Click **Start Navigation** in the Nav2 panel. The rover will visit every point in order!
