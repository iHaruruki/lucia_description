import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command


class RobotDescriptionLoader():

    def __init__(self):
        self.robot_description_path = os.path.join(
            get_package_share_directory('raspimouse_description'),
            'urdf',
            'raspimouse.urdf.xacro')
        self.lidar = 'none'
        self.lidar_frame = 'laser'
        self.use_gazebo = 'false'
        self.use_rgb_camera = 'false'
        self.camera_downward = 'false'
        self.gz_control_config_package = ''
        self.gz_control_config_file_path = ''

    def load(self):
        return Command([
                'xacro ',
                self.robot_description_path,
                ' lidar:=', self.lidar,
                ' lidar_frame:=', self.lidar_frame,
                ' use_gazebo:=', self.use_gazebo,
                ' use_rgb_camera:=', self.use_rgb_camera,
                ' camera_downward:=', self.camera_downward,
                ' gz_control_config_package:=', self.gz_control_config_package,
                ' gz_control_config_file_path:=', self.gz_control_config_file_path
                ])