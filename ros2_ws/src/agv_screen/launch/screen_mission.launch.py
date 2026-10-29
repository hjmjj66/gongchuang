"""Launch mission waiting for the screen; hardware executors remain separate."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mission_config = os.path.join(get_package_share_directory('agv_mission'), 'config', 'mission.yaml')
    screen_config = os.path.join(get_package_share_directory('agv_screen'), 'config', 'screen.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('mission_config', default_value=mission_config),
        DeclareLaunchArgument('screen_config', default_value=screen_config),
        Node(package='agv_mission', executable='mission_node', name='mission_node', output='screen',
             parameters=[LaunchConfiguration('mission_config'), {'screen_controlled': True}]),
        Node(package='agv_screen', executable='screen_node', name='screen_node', output='screen',
             parameters=[LaunchConfiguration('screen_config')]),
    ])
