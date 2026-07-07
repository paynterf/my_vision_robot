#!/bin/bash

echo "=== Sourcing ROS environments ==="
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo "=== Starting depth frame capture ==="
python3 ~/my_vision_robot/software/scripts/save_depth_frame.py