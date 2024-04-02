#!/bin/bash

. /opt/ros/humble/setup.bash

mkdir -p src
git clone --recursive --depth 1 https://github.com/cyberbotics/webots_ros2.git src/webots_ros2
git clone --recursive --depth 1 -b humble https://github.com/ros-planning/navigation2 src/navigation2
mv src/navigation2/nav2_mppi_controller /src
rm -rf src/navigation2
apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
chown -R deweykai /home/webots_ws/
rm -rf /var/lib/apt/lists/*
