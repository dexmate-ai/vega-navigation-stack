#!/bin/bash

# PTP Clock Startup Script for LiDAR Synchronization
# This script starts the PTP clock for LiDAR timestamp synchronization
# Usage: sudo ./start_ptp_clock.sh <network_interface>
# Example: sudo ./start_ptp_clock.sh enp3s0

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Error: This script must be run as root (use sudo)"
    echo "Usage: sudo $0 <network_interface>"
    echo "Example: sudo $0 enp3s0"
    exit 1
fi

# Check if network interface argument is provided
if [ -z "$1" ]; then
    echo "Error: Network interface name is required"
    echo "Usage: sudo $0 <network_interface>"
    echo "Example: sudo $0 enp3s0"
    echo ""
    echo "Available network interfaces:"
    ip link show | grep -E '^[0-9]+:' | awk -F': ' '{print "  " $2}'
    exit 1
fi

NETWORK_INTERFACE=$1

echo "Starting PTP clock on interface: $NETWORK_INTERFACE"
echo "Make sure your LiDAR is connected to this interface!"
echo ""

# Change to ptp_clock directory
cd ${SCRIPT_DIR}/system_cfgs/ptp_clock

# Start ptp4l in the background
echo "Starting ptp4l..."
ptp4l -i $NETWORK_INTERFACE -ml 6 -f automotive-master.cfg &
PTP4L_PID=$!
echo "ptp4l started with PID: $PTP4L_PID"

# Wait a moment for ptp4l to initialize
sleep 2

# Start phc2sys in the background
echo "Starting phc2sys..."
phc2sys -a -rr &
PHC2SYS_PID=$!
echo "phc2sys started with PID: $PHC2SYS_PID"

echo ""
echo "PTP clock started successfully!"
echo "  ptp4l PID: $PTP4L_PID"
echo "  phc2sys PID: $PHC2SYS_PID"
echo ""
echo "To stop the PTP clock, run:"
echo "  sudo kill $PTP4L_PID $PHC2SYS_PID"
echo ""
echo "Or to stop all PTP processes:"
echo "  sudo killall ptp4l phc2sys"

