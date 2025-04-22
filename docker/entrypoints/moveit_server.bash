#!/bin/bash

source /root/.bashrc
source /colcon_ws/install/setup.bash
ros2 launch spot_moveit_config move_group.launch.py
