import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'lucia_description'  # package name
    urdf_file_name = 'turtlebot3_burger.urdf'    # URDF file name

    # パッケージの共有ディレクトリを取得
    urdf_file_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file_name
    )

    # URDFの内容を取得
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

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
            parameters=[{'robot_description': robot_desc}]
        )
        #Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    name='rviz2',
        #    output='screen'
        #)
    ])
