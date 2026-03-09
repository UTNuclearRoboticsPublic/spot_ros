#!/bin/bash

source /root/.bashrc
source /colcon_ws/install/setup.bash
ros2 launch spot_navigation slam.launch.py
