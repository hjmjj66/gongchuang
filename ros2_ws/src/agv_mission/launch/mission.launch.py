"""Coordinator only. Real task executors are started separately."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_mission(context):
    overrides = {name: LaunchConfiguration(name).perform(context)
                 for name in ('mode', 'start_area', 'finish_area')
                 if LaunchConfiguration(name).perform(context)}
    parameters = [LaunchConfiguration('config')]
    if overrides:
        parameters.append(overrides)
    return [Node(package='agv_mission', executable='mission_node', name='mission_node',
                 output='screen', parameters=parameters)]


def generate_launch_description():
    config = os.path.join(get_package_share_directory('agv_mission'), 'config', 'mission.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('mode', default_value='', description='Optional override of YAML mode'),
        DeclareLaunchArgument('start_area', default_value='', description='Optional override of YAML start_area'),
        DeclareLaunchArgument('finish_area', default_value='', description='Optional override of YAML finish_area'),
        OpaqueFunction(function=_launch_mission),
    ])
