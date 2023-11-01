import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    boden_dir = get_package_share_directory('boden_bringup')
    launch_dir = os.path.join(boden_dir, 'launch')
    params_dir = os.path.join(boden_dir, 'config')
    nav2_params = os.path.join(params_dir, 'nav2_no_map_params.yml')
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key='', param_rewrites='', convert_types=True
    )

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ekf_launch.py')),
    )

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': configured_params,
            'autostart': 'True',
        }.items(),
    )

    ld = LaunchDescription()
    
    ld.add_action(robot_localization_cmd)
    ld.add_action(nav2_cmd)

    return ld