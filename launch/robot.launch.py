import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('lucia_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'lucia.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',    # fixed typo from before
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'publish_robot_description': True
            }],
        ),
    ])
