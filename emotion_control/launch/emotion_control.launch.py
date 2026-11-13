from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('emotion_control'),
                          'config', 'emotion.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('show_image', default_value='true'),
        Node(package='emotion_control', executable='emotion_control', output='screen',
             parameters=[LaunchConfiguration('config'), {
                 'camera_index': ParameterValue(LaunchConfiguration('camera_index'), value_type=int),
                 'show_image': ParameterValue(LaunchConfiguration('show_image'), value_type=bool),
             }]),
    ])
