#!/bin/bash
# RoboGrasp-Vision 键盘交互启动脚本
# 用法: ./scripts/mvp_interactive.sh
#
# 启动 mvp_demo launch，提供键盘菜单，按键触发 mock_perception 检测
#
# 与 piper_moveit_bridge 联用:
#   终端1: ros2 launch piper_highlevel piper_moveit_bridge.launch.py
#   终端2: ./scripts/mvp_interactive.sh

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
echo " 1: cube      (0.35,  0.05, 0.02)"
echo " 2: cylinder  (0.35, -0.10, 0.02)"
echo " 3: box       (0.25,  0.10, 0.02)"
echo " q: quit & shutdown"
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
