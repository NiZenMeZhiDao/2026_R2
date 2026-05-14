import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bridge_dir = get_package_share_directory('slam_odin_bridge')
    odin_dir = get_package_share_directory('odin_ros_driver')

    default_bridge_config = os.path.join(bridge_dir, 'config', 'param.yaml')
    default_mount_config = os.path.join(bridge_dir, 'config', 'odin_mount.yaml')
    default_odin_config = os.path.join(odin_dir, 'config', 'control_command.yaml')
    odin_launch = os.path.join(odin_dir, 'launch', 'odin1_ros2.launch.py')

    bridge_config_arg = DeclareLaunchArgument(
        'bridge_config_file',
        default_value=default_bridge_config,
        description='Path to the Odin bridge parameter file.',
    )
    mount_config_arg = DeclareLaunchArgument(
        'mount_config_file',
        default_value=default_mount_config,
        description='Path to the Odin mounting extrinsic parameter file.',
    )
    odin_config_arg = DeclareLaunchArgument(
        'odin_config_file',
        default_value=default_odin_config,
        description='Path to the Odin driver control_command.yaml file.',
    )
    pcd_path_arg = DeclareLaunchArgument(
        'pcd_path',
        default_value=os.path.expanduser('~/2026_R2/slam_odin/map.pcd'),
        description='Path to the static PCD map published on /odin1/map.',
    )

    odin_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(odin_launch),
        launch_arguments={
            'config_file': LaunchConfiguration('odin_config_file'),
        }.items(),
    )

    bridge_node = Node(
        package='slam_odin_bridge',
        executable='odin_localization_bridge',
        name='odin_localization_bridge',
        output='screen',
        parameters=[
            LaunchConfiguration('bridge_config_file'),
            LaunchConfiguration('mount_config_file'),
        ],
    )

    map_node = Node(
        package='slam_odin_bridge',
        executable='pcd_map_publisher',
        name='pcd_map_publisher',
        output='screen',
        parameters=[
            LaunchConfiguration('bridge_config_file'),
            {'pcd_path': LaunchConfiguration('pcd_path')},
        ],
    )

    return LaunchDescription([
        bridge_config_arg,
        mount_config_arg,
        odin_config_arg,
        pcd_path_arg,
        odin_driver,
        bridge_node,
        map_node,
    ])
