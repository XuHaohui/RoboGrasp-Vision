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

# Recommended locale setting
export LC_NUMERIC=en_US.UTF-8

echo "RoboGrasp-Vision workspace loaded"
echo "  ROS_DISTRO=$ROS_DISTRO"
echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
