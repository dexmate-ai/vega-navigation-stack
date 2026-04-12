#!/bin/bash

# DDS Setup Script for ROS2 with CycloneDDS
# This script exports the necessary environment variables for optimal DDS communication
#
# IMPORTANT: This script must be SOURCED, not executed!
# Usage: source ./dds_setup.sh
#    or: . ./dds_setup.sh

# Check if script is being sourced or executed
if [ "${BASH_SOURCE[0]}" -ef "$0" ]; then
    echo "ERROR: This script must be SOURCED, not executed!"
    echo ""
    echo "Please run it as:"
    echo "  source ./dds_setup.sh"
    echo "or:"
    echo "  . ./dds_setup.sh"
    echo ""
    echo "Running with ./ will not set the environment variables in your current shell."
    exit 1
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Set RMW implementation to CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Set CycloneDDS configuration file path
export CYCLONEDDS_URI=${SCRIPT_DIR}/system_cfgs/cyclone_dds_setup.xml

echo "DDS Setup Complete:"
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "  CYCLONEDDS_URI=${CYCLONEDDS_URI}"

