from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=['home/peach/ros2_ws/src/lucia_slam_navigation/lucia_localization/config/ekf.yaml'],
        ),
    ])