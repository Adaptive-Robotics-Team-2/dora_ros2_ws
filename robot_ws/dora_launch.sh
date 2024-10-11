#!/bin/bash

# Exit immediately if any command exits with a non-zero status
set -e

# Change permissions for serial devices before proceeding
echo "Setting permissions for /dev/ttyS0 and /dev/ttyUSB0..."
sudo chmod 666 /dev/ttyS0
sudo chmod 666 /dev/ttyUSB0

# Build the workspace
echo "Building workspace..."
colcon build --packages-select dora_robot_control

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

# Launch the robot
echo "Launching dora_robot..."
ros2 launch dora_robot dora_robot_launch.py 
