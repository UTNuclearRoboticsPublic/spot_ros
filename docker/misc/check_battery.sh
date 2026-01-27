#!/bin/bash

source /opt/ros/humble/setup.bash
source /colcon_ws/install/local_setup.bash
ros2 topic echo /spot_driver/status/battery_states
