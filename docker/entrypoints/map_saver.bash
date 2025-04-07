#!/bin/bash

source /root/.bashrc
source /colcon_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f /tmp/spot_nav_map --ros-args -p free_thresh_default:=0.15 -r map:=spot_nav/map
