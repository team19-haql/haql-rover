#!/bin/bash

. /home/webots_ws/install/setup.bash
sudo apt update
rosdep update --rosdistro=humble
rosdep install --from-paths /home/ws/src --ignore-src -y --rosdistro=humble
chmod +x /home/webots_ws/install/webots_ros2_driver/lib/webots_ros2_driver/ros2_supervisor.py 
sudo chown -R deweykai /home/ws/