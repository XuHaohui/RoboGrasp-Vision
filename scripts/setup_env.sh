#!/bin/bash
# RoboGrasp-Vision environment setup script
# Usage: source scripts/setup_env.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Source ROS2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "[WARN] /opt/ros/humble/setup.bash not found"
fi

# Source this workspace
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
else
    echo "[WARN] Workspace not built. Run: colcon build --symlink-install"
fi

# Optional: use CycloneDDS for better communication
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/piper_conrtrol/scripts/cyclonedds.xml"

# Recommended locale setting
export LC_NUMERIC=en_US.UTF-8

# Workaround: bypass broken ROS 2 daemon (Python 3.10 importlib.metadata bug)
ros2() {
    case "$1" in
        topic|node|service|param|action|interface|pkg|lifecycle|bag)
            command ros2 "$@" --no-daemon ;;
        *)
            command ros2 "$@" ;;
    esac
}

echo "RoboGrasp-Vision workspace loaded"
echo "  ROS_DISTRO=$ROS_DISTRO"
echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
