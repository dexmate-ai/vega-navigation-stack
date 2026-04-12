#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR

## Setup DDS environment variables
# source ./dds_setup.sh

# Start PTP clock for LiDAR synchronization
# Note: Set LIDAR_NETWORK_INTERFACE to your network interface (e.g., enp3s0)
# Run these commands in a separate terminal before starting the system:
#   cd system_cfgs/ptp_clock
#   sudo ptp4l -i ${LIDAR_NETWORK_INTERFACE:-enp3s0} -S -ml 6 -f automotive-master.cfg &
#   sudo phc2sys -a -rr &

source ./install/setup.bash
ros2 launch vehicle_simulator system_real_robot_rs_airy.launch.py
