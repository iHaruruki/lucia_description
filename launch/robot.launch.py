import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'lucia_description'  # package name
    urdf_file_name = 'lucia.urdf'    # URDF file name

    # build the URDF string by xacro
    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
        FindPackageShare('lucia_description'), 'urdf', 'lucia.urdf'
        ])
    ])

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
            name='robot_state_pushlisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'publish_robot_description': True}
            ]
        )
        #Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    name='rviz2',
        #    output='screen'
        #)
    ])
