#!/bin/bash

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPS_DIR="$(dirname "$SCRIPT_DIR")/maps"

# Create the maps directory if it doesn't exist
mkdir -p "$MAPS_DIR"

# Set timestamp for unique map name or use provided name
if [ $# -eq 0 ]; then
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    MAP_NAME="map_$TIMESTAMP"
else
    MAP_NAME="$1"
fi

# Save the map
echo "Saving map to $MAPS_DIR/$MAP_NAME"

# For PGM/YAML map format
echo "Saving standard map format (PGM/YAML)..."
ros2 run nav2_map_server map_saver_cli -f "$MAPS_DIR/$MAP_NAME" --ros-args -p save_map_timeout:=10.0

# For SLAM Toolbox serialized map format
echo "Saving serialized map for SLAM Toolbox..."
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$MAPS_DIR/$MAP_NAME'}"

echo "Maps saved to $MAPS_DIR/$MAP_NAME"
echo "  - Standard map format: $MAPS_DIR/$MAP_NAME.pgm and $MAPS_DIR/$MAP_NAME.yaml"
echo "  - Serialized map for SLAM Toolbox: $MAPS_DIR/$MAP_NAME (no extension)" 