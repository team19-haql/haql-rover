import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    description_dir = get_package_share_directory('boden_description')
    
    # read the urdf into a string
    urdf = os.path.join(description_dir, 'urdf', 'assem_ros.urdf')
    with open(urdf, 'r') as file:
        urdf = file.read()

    # Launch the robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf}]
    )

    # Launch the joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': [urdf]}]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        # joint_state_publisher_node
    ])

if __name__ == '__main__':
    generate_launch_description()