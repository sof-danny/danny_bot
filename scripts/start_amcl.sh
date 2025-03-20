#!/bin/bash

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Run the AMCL localization launch file
ros2 launch danny_bot amcl_localization.launch.py 