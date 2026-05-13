import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('slam_odin_bridge')
    default_config = os.path.join(package_dir, 'config', 'param.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the Odin localization bridge parameter file.',
    )
    pcd_path_arg = DeclareLaunchArgument(
        'pcd_path',
        default_value=os.path.expanduser('~/2026_R2/slam_odin/map.pcd'),
        description='Path to the static PCD map published on /odin1/map.',
    )

    bridge_node = Node(
        package='slam_odin_bridge',
        executable='odin_localization_bridge',
        name='odin_localization_bridge',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    map_node = Node(
        package='slam_odin_bridge',
        executable='pcd_map_publisher',
        name='pcd_map_publisher',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'pcd_path': LaunchConfiguration('pcd_path')},
        ],
    )

    return LaunchDescription([
        config_arg,
        pcd_path_arg,
        bridge_node,
        map_node,
    ])
