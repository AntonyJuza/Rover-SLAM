#!/usr/bin/env bash
# ==============================================================================
# Save Live Explored SLAM Map to File
# ==============================================================================

MAP_NAME=${1:-"explored_room_map"}
MAP_DIR="/home/juza/rover_slam_ws/maps"

mkdir -p "$MAP_DIR"
echo "[INFO] Saving active SLAM map to ${MAP_DIR}/${MAP_NAME}..."

ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/${MAP_NAME}" --ros-args -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0

if [ -f "${MAP_DIR}/${MAP_NAME}.yaml" ]; then
    echo "[SUCCESS] Map saved successfully!"
    echo "  - YAML: ${MAP_DIR}/${MAP_NAME}.yaml"
    echo "  - PGM:  ${MAP_DIR}/${MAP_NAME}.pgm"
else
    echo "[ERROR] Failed to save map. Ensure SLAM Toolbox is publishing to /map."
fi
