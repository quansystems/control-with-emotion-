"""Semantic navigation; start localization/Nav2 separately as in the main README."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from robot_node.semantic_map import default_regions_file


def generate_launch_description():
    share = FindPackageShare('robot_node')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('regions_file', default_value=default_regions_file()),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='robot_node', executable='voice_navigation', output='screen',
             parameters=[{
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'regions_file': LaunchConfiguration('regions_file'),
             }]),
        Node(package='rviz2', executable='rviz2', name='voice_rviz', output='screen',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'voice_navigation.rviz'])],
             parameters=[{'use_sim_time': ParameterValue(
                 LaunchConfiguration('use_sim_time'), value_type=bool)}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
