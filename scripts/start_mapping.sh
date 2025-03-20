#!/bin/bash

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Run the SLAM launch file with explicit parameter override for mapping mode
ros2 launch danny_bot slam.launch.py slam_mode:=mapping

# Alternatively, you can verify the mode was set correctly
# echo "Current SLAM mode is:"
# sleep 2  # Wait for system to initialize
# ros2 param get /slam_toolbox slam_toolbox.ros__parameters.mode 