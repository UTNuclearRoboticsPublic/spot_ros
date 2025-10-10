#!/bin/bash

source /root/.bashrc
source /colcon_ws/install/setup.bash
ros2 launch spot_bringup bringup.launch.py hostname:=192.168.50.3 dock_id:="${DOCK_ID:-520}" launch_velodyne:="${LAUNCH_VELODYNE:-true}" "$@"
