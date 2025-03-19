#!/bin/bash

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Run the AMCL localization launch file
ros2 launch mobile_dd_robot amcl_localization.launch.py 