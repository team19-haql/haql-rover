#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots Tesla driver."""

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('boden_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    robot_localization_params = os.path.join(package_dir, 'config', 'localization.yml')
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node_odom',
            parameters=[
                {'use_sim_time': use_sim_time },
                robot_localization_params],
            remappings=[('odometry/filtered', 'odometry/local'),
                        ('imu/data', 'imu')]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node_map',
            parameters=[
                {'use_sim_time': use_sim_time },
                robot_localization_params],
            remappings=[('odometry/filtered', 'odometry/global'),
                        ('imu/data', 'imu')]
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            parameters=[
                {'use_sim_time': use_sim_time },
                robot_localization_params],
            remappings=[('imu/data', 'imu'),
                        ('gps/fix', 'gps'), 
                        ('gps/filtered', 'gps/filtered'),
                        ('odometry/gps', 'odometry/gps'),
                        ('odometry/filtered', 'odometry/global')]
        )
    ])
