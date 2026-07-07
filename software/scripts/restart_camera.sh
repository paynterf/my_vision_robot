#!/bin/bash

echo "=== Killing old camera processes ==="
pkill -f depthai_ros_driver
pkill -f camera_node
pkill -f rqt_image_view
sleep 3

echo "=== Sourcing environments ==="
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo "=== Starting camera ==="
ros2 launch my_vision_robot oak_camera.launch.py