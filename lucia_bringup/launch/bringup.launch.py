import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    rplidar_launch = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{'serial_port': '/dev/ttyUSB0',  # 実際のポートに変更
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True}]
    )
            
    laser_filter_node = Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                    PathJoinSubstitution([
                    get_package_share_directory("laser_filters"),
                    "examples", "box_filter_example.yaml",
            ])],
            remappings=[('/scan','/rplidar_ros/scan'),('/scan_filtered','/scan')],
    )

    tf2_base_link_ldlidar_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'rplidar_base']
    )

    bringup_dir = get_package_share_directory('lucia_bringup')
    cartographer_config_dir = os.path.join(bringup_dir, 'config')
    configuration_basename = 'noimu_lds_2d.lua'

    use_sim_time = False
    resolution = '0.05'
    publish_period_sec = '1.0'

    cartographer_node = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]
    )

    occupancy_grid_node = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
    )

    rviz2_config = os.path.join(
        get_package_share_directory('lucia_bringup'),
        'rviz',
        'config.rviz'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[["-d"], [rviz2_config]]
    )

    ld = LaunchDescription()
    #ld.add_action(rplidar_launch)
    ld.add_action(laser_filter_node)
    ld.add_action(tf2_base_link_ldlidar_base)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(rviz2_node)
    return ld