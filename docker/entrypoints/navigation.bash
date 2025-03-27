#!/bin/bash

# Source ROS and our package
source /opt/ros/${ROS_DISTRO}/setup.bash
source /colcon_ws/install/local_setup.bash

# Check to see if a custom map path has been provided
if [[ ! -z "$SPOT_NAV_MAP" ]]; then
    # Replace the original image name with the mounted version
    yq -yi '.image = "user_map.pgm"' $MAP_PATH
else
    # Otherwise we use the default AHG map
    MAP_PATH=$(ros2 pkg prefix --share spot_navigation)/map/ahg.yaml
fi

# Launch both localization and navigation and capture their process IDs
ros2 launch spot_navigation amcl.launch.py config:=$(ros2 pkg prefix --share spot_navigation)/config/spot.yaml map:=$MAP_PATH &
AMCL_PID=($!)
ros2 launch spot_navigation bringup_launch.py config:=$(ros2 pkg prefix --share spot_navigation)/config/spot.yaml &
NAV_PID=($!)

# Set up a signal handler to kill both processes on interrupt
trap 'kill $AMCL_PID;kill $NAV_PID' SIGINT

# Wait for both processes to finish
wait $AMCL_PID
wait $NAV_PID
