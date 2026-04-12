#!/bin/bash
set -e

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source workspace if it exists
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
    source "${WORKSPACE}/install/setup.bash"
    echo "Workspace sourced successfully"
fi

# Set up CycloneDDS environment
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml

# Try to set kernel receive buffer for CycloneDDS (requires host network mode)
# This will fail silently if not possible, but the system should still work
if [ -w /proc/sys/net/core/rmem_max ]; then
    sysctl -w net.core.rmem_max=2147483647 2>/dev/null || echo "Note: Could not set net.core.rmem_max. For optimal performance with large LiDAR messages, run on host: sudo sysctl -w net.core.rmem_max=2147483647"
fi

echo "=========================================="
echo "Autonomy Stack Vega - Docker Environment"
echo "ROS2 Jazzy Jalisco"
echo "RMW: ${RMW_IMPLEMENTATION}"
echo "Workspace: ${WORKSPACE}"
echo "=========================================="
echo ""
echo "To build the workspace:"
echo "  cd ${WORKSPACE}"
echo "  colcon build --symlink-install"
echo ""
echo "To build specific packages:"
echo "  colcon build --symlink-install --packages-select rslidar_msg rslidar_sdk"
echo ""
echo "=========================================="

# Execute the command passed to the container
exec "$@"
