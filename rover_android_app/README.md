# Rover Android Mission Control App

A modern, responsive Android app (Progressive Web App) for real-time mobile control of your custom ROS 2 Rover.

---

## 🌟 Key Features

| Feature | Description |
| :--- | :--- |
| **🗺️ Real-time Canvas Map** | High-performance HTML5 canvas rendering of `/map`, global plan path (`/plan`), and optional LiDAR scans (`/scan`). |
| **🎯 2D Pose Estimation** | Touch & drag an arrow directly on the map to publish `/initialpose` for AMCL alignment. |
| **📍 2D Nav Goal** | Touch & drag to dispatch autonomous navigation goals (`/goal_pose`) to Nav2. |
| **🚩 Waypoint Missions** | Create, reorder, and execute multi-waypoint missions with custom pause times. |
| **🕹️ Virtual Touch Joystick** | Analog touchscreen thumbstick for manual testing and driving (`/cmd_vel_smoothed`). |
| **🛑 Emergency Stop** | Instant one-tap button to zero all velocities and cancel navigation. |
| **📲 PWA Installation** | Installable onto your Android Home Screen as a standalone, fullscreen native-like app. |

---

## 🚀 How to Run (1 Simple Command)

On your Linux workstation (or on the Raspberry Pi):

```bash
python3 /home/juza/rover_slam_ws/rover_android_app/start_app.py
```

This script will automatically:
1. Detect your local Wi-Fi IP (e.g., `192.168.1.54`).
2. Start the ROS 2 WebSocket bridge on port `9090` (if not already running).
3. Host the web app on port `8000`.
4. **Print an ASCII QR code directly into your terminal!**

---

## 📱 How to Open on Your Android Phone

1. **Ensure your Android phone is connected to the same Wi-Fi** as your laptop / Raspberry Pi.
2. **Scan the terminal QR code** using your Android phone's camera, OR open Chrome on Android and navigate to:
   ```text
   http://192.168.1.54:8000
   ```
   *(Replace with the IP displayed by `start_app.py`)*

### 📲 Install to Android Home Screen (Optional Fullscreen App)
1. In Google Chrome on your Android phone, tap the **three dots menu** (⋮) at top-right.
2. Select **"Add to Home screen"** or **"Install app"**.
3. A custom **Rover** icon will be added to your app drawer and home screen. Opening it gives you a clean fullscreen interface with no browser URL bar!

---

## 🎮 How to Use the Tools

### 1. 🎯 2D Pose Estimation (AMCL)
1. Tap the **2D Pose** tool on the bottom toolbar.
2. On the map, touch the rover's physical starting position and drag in the direction the rover is facing.
3. Release your finger — the green orientation arrow will publish to `/initialpose` and AMCL particles will align immediately!

### 2. 📍 2D Nav Goal
1. Tap the **Nav Goal** tool on the bottom toolbar.
2. Touch your desired target location and drag an arrow in the desired heading.
3. Release — Nav2 will compute a global path (cyan line) and the rover will begin driving!

### 3. 🚩 Waypoint Missions
1. Tap the **Waypoints** tool to open the Mission Drawer.
2. Tap on the map to add destinations (Point 1, Point 2, Point 3...).
3. Alternatively, click **"➕ Current Pose"** while driving to bookmark spots.
4. Customize wait durations, then tap **"▶️ Start Waypoint Mission"**.
5. The app will sequence through each waypoint with arrival countdowns!

### 4. 🕹️ Virtual Joystick (Testing & Manual Teleop)
1. Tap the **Drive** tool on the bottom toolbar.
2. An analog thumbstick will appear on the bottom-right.
3. Slide forward to drive ahead, slide left/right to steer.
4. Release to stop.
5. In Settings (⚙️), you can tune the maximum manual speed limit (default: 0.12 m/s).
