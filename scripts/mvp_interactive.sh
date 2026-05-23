#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Source 环境
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
else
    echo "[ERROR] Workspace not built. Run: colcon build --symlink-install"
    exit 1
fi

echo "========================================"
echo " RoboGrasp-Vision Interactive Controller"
echo "========================================"
echo ""

NODE_NAME="/mock_perception"

echo "[1/2] Starting mvp_demo launch in background..."
ros2 launch robograsp_vision_bringup mvp_demo.launch.py &
LAUNCH_PID=$!

echo "[2/2] Waiting for nodes to become ready..."
sleep 3

echo ""
echo "========================================"
echo " Use keyboard to publish mock perception"
echo "----------------------------------------"
echo " #  name       shape      position (x, y, z)     size (w x d x h) m"
echo "---  --------  ---------  ---------------------  --------------------"
echo " 1:  cube      [cube]    (0.35,  0.05, 0.20)    0.04 x 0.04 x 0.04"
echo " 2:  cylinder  [cylinder] (0.35, -0.10, 0.20)   0.04 x 0.04 x 0.06"
echo " 3:  box       [box]     (0.25,  0.10, 0.20)    0.05 x 0.05 x 0.05"
echo " q:  quit & shutdown"
echo "========================================"

trap 'echo ""; echo "Shutting down..."; ros2 param set '$NODE_NAME' trigger_index 0 2>/dev/null; kill $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null; exit 0' INT TERM

while true; do
    read -r -p "> " key

    case "$key" in
        1|2|3)
            echo "Publishing object #$key..."
            ros2 param set "$NODE_NAME" trigger_index "$key"
            ;;
        q|Q)
            echo "Quitting..."
            ros2 param set "$NODE_NAME" trigger_index 0 2>/dev/null
            kill $LAUNCH_PID 2>/dev/null
            wait $LAUNCH_PID 2>/dev/null
            break
            ;;
        "")
            ;;
        *)
            echo "Invalid key. Press 1-3 or q to quit."
            ;;
    esac
done
