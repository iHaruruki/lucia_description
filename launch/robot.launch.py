import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # locate your URDF
    pkg_share = get_package_share_directory('lucia_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'lucia.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # create the launch description
    ld = LaunchDescription()

    # joint_state_publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )
    ld.add_action(joint_state_publisher)

    # robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'publish_robot_description': True,
        }],
    )
    # if you want to delay the state publisher by 1 second:
    delayed_robot_state = TimerAction(
        period=1.0,
        actions=[robot_state_publisher]
    )
    ld.add_action(delayed_robot_state)

    # finally, return the LaunchDescription!
    return ld
