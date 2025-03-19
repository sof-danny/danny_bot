#!/bin/bash

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Check if slam_toolbox is running
if ! ros2 topic list | grep -q "/slam_toolbox"; then
    echo "Error: slam_toolbox does not appear to be running."
    echo "Start it first with './start_mapping.sh' or './start_localization.sh'"
    exit 1
fi

# Wait for parameters to be available
echo "Checking SLAM Toolbox mode..."
sleep 1

# Get the current mode
MODE=$(ros2 param get /slam_toolbox slam_toolbox.ros__parameters.mode 2>/dev/null)

if [ $? -ne 0 ]; then
    echo "Error: Could not get slam_toolbox mode parameter."
    echo "Parameter might not be available yet or slam_toolbox isn't running."
    exit 1
fi

echo "Current SLAM Toolbox mode: $MODE"

# Check if we're in localization mode, and if so, display the map file
if [[ "$MODE" == *"localization"* ]]; then
    echo "In localization mode - checking map file..."
    MAP_FILE=$(ros2 param get /slam_toolbox slam_toolbox.ros__parameters.map_file_name 2>/dev/null)
    if [ $? -eq 0 ]; then
        echo "Using map file: $MAP_FILE"
    else
        echo "Could not determine map file."
    fi
fi 