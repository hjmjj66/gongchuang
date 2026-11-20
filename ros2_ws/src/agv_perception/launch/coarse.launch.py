from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = str(Path(get_package_share_directory('agv_perception'))/'config/coarse.yaml')
    storage = str(Path(get_package_share_directory('agv_perception'))/'config/storage.yaml')
    raw = str(Path(get_package_share_directory('agv_perception'))/'config/raw.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('raw_config', default_value=raw),
        DeclareLaunchArgument('config', default_value=default),
        DeclareLaunchArgument('storage_config', default_value=storage),
        Node(package='agv_perception', executable='coarse_node', output='screen',
             parameters=[{'config': LaunchConfiguration('config'),
                          'storage_config': LaunchConfiguration('storage_config'),
                          'raw_config': LaunchConfiguration('raw_config')}]),
    ])
