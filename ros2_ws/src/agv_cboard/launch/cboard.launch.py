"""Start the C-board chassis bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    parameters = [LaunchConfiguration('config')]
    serial_port = LaunchConfiguration('serial_port').perform(context)
    if serial_port:
        parameters.append({'serial_port': serial_port})
    return [
        Node(
            package='agv_cboard', executable='cboard_bridge_node',
            name='cboard_bridge_node', output='screen',
            parameters=parameters,
        ),
    ]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('agv_cboard'), 'config', 'cboard.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('serial_port', default_value='',
                              description='Optional override; otherwise use YAML serial_port'),
        OpaqueFunction(function=_launch_setup),
    ])
