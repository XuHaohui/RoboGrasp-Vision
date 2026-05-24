#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

if [ -f "$WORKSPACE_DIR/scripts/setup_env.sh" ]; then
    source "$WORKSPACE_DIR/scripts/setup_env.sh"
else
    echo "[ERROR] setup_env.sh not found"
    exit 1
fi

echo "=============================="
echo " RoboGrasp-Vision Camera Detector"
echo "=============================="
echo ""

echo "[1/2] Launching camera detection pipeline..."
ros2 launch robograsp_vision_bringup camera_detect.launch.py &
LAUNCH_PID=$!

echo "[2/2] Waiting for nodes to become ready..."
sleep 3

echo ""
python3 "$SCRIPT_DIR/camera_interactive.py"

echo ""
echo "Shutting down..."
kill $LAUNCH_PID 2>/dev/null
wait $LAUNCH_PID 2>/dev/null
