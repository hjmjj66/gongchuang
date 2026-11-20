"""Start the material diagnostic with a shared ROS parameter YAML."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('agv_perception'),'config','material.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=default_config),
        Node(package='agv_perception',executable='material_debug_node',name='material_debug_node',
             output='screen',parameters=[LaunchConfiguration('config')]),
    ])
