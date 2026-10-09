# Frontier-Based Autonomous Exploration Guide

This guide explains how to run **Frontier-Based Autonomous Exploration** on your rover using:
1. **SLAM Toolbox**: Builds the live map from LiDAR & Odometry and publishes `map -> odom`.
2. **Nav2 (SLAM Mode)**: Generates trajectories and drives the rover (without AMCL or pre-saved maps).
3. **`frontier_exploration_ros2`**: Finds the boundary between known free space and unexplored unknown space, choosing optimal reachable frontier targets until the environment is 100% mapped.

---

## 🌿 Git Branch Structure

As requested, all exploration changes are isolated in a dedicated Git branch:

- **`main`**: Production branch containing static map navigation, AMCL localization, waypoint mission scripts, and the Android PWA.
- **`autonomous-exploration`**: Exploration branch containing SLAM-mode Nav2 parameters, frontier exploration configs, and unified launch scripts.

To switch branches anytime:
```bash
# Switch to exploration:
git checkout autonomous-exploration

# Switch back to AMCL / Waypoint missions:
git checkout main
```

---

## 🛠️ Configuration & Launch Files

| File | Location | Description |
| :--- | :--- | :--- |
| [`frontier_params.yaml`](file:///home/juza/rover_slam_ws/rover_bringup/config/frontier_params.yaml) | `config/frontier_params.yaml` | Explorer node parameters scaled for the rover's 0.18m radius and indoor environments. |
| [`nav2_slam_params.yaml`](file:///home/juza/rover_slam_ws/rover_bringup/config/nav2_slam_params.yaml) | `config/nav2_slam_params.yaml` | Nav2 stack without AMCL/map_server; costmaps subscribe directly to live `/map` with reverse-enabled DWB planner. |
| [`autonomous_exploration.launch.py`](file:///home/juza/rover_slam_ws/rover_bringup/launch/autonomous_exploration.launch.py) | `launch/autonomous_exploration.launch.py` | Unified launch file starting Nav2 and `frontier_explorer`. |
| [`save_explored_map.sh`](file:///home/juza/rover_slam_ws/rover_bringup/scripts/save_explored_map.sh) | `scripts/save_explored_map.sh` | One-click script to save the completed SLAM map. |

---

## 🚀 Step-by-Step Autonomous Exploration Workflow

### Step 1: Launch Rover Hardware + SLAM Toolbox (on the Raspberry Pi)
In a terminal on the Raspberry Pi:
```bash
ros2 launch rover_bringup bringup.launch.py use_slam:=true left_trim:=1.0 right_trim:=1.0 use_imu:=true
```
*This starts the LiDAR, Cytron motor drivers, differential odometry, and SLAM Toolbox in online async mapping mode (`map -> odom`).*

---

### Step 2: Launch Nav2 & Frontier Explorer (on the Raspberry Pi)
In a second terminal on the Raspberry Pi:
```bash
ros2 launch rover_bringup autonomous_exploration.launch.py
```
*This starts Nav2 (controller, planner, recovery behaviors) and `frontier_explorer` (`frontier_exploration_ros2`). The explorer will immediately begin scanning the live map for frontiers and dispatching navigation goals.*

---

### Step 3: Monitor Live Exploration in RViz (on Workstation)
On your laptop:
```bash
rviz2 -d $(ros2 pkg prefix rover_description)/share/rover_description/rviz/rover_slam.rviz
```
In RViz, add the following visual displays:
- **Map**: Topic `/map` (Shows the growing map in real time).
- **Frontier Markers**: Add **MarkerArray** on topic `/explore/frontiers` (Shows clusters of unexplored boundary points).
- **Target Goal**: Add **Pose** on topic `/explore/selected_frontier` (Shows the active exploration target).
- **Global Path**: Topic `/plan` (Shows the planned path to the frontier).

---

### Step 4: Save the Newly Explored Map
Once the rover has explored the room (or when you are satisfied with the map coverage), run:
```bash
/home/juza/rover_slam_ws/rover_bringup/scripts/save_explored_map.sh my_explored_room
```
This saves:
- `maps/my_explored_room.yaml`
- `maps/my_explored_room.pgm`

You can then load this map anytime in the `main` branch for AMCL localization and waypoint missions!
