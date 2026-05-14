import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    runtime_dir = get_package_share_directory('robot_runtime')
    bridge_dir = get_package_share_directory('slam_odin_bridge')
    odin_dir = get_package_share_directory('odin_ros_driver')

    runtime_launch = os.path.join(runtime_dir, 'launch', 'runtime_bottom_layer.launch.py')
    odin_launch = os.path.join(bridge_dir, 'launch', 'odin_external_bringup.launch.py')
    default_odin_config = os.path.join(odin_dir, 'config', 'control_command.yaml')
    default_mount_config = os.path.join(bridge_dir, 'config', 'odin_mount.yaml')

    bridge_config_arg = DeclareLaunchArgument(
        'bridge_config_file',
        default_value=os.path.join(bridge_dir, 'config', 'param.yaml'),
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
        default_value=os.path.expanduser('~/2026_R2/map.pcd'),
        description='Path to the static PCD map published on /odin1/map.',
    )
    debug_period_arg = DeclareLaunchArgument(
        'debug_period_sec',
        default_value='0.5',
        description='Runtime debug topic publish period in seconds; <=0 disables it.',
    )

    runtime = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(runtime_launch),
        launch_arguments={
            'debug_period_sec': LaunchConfiguration('debug_period_sec'),
        }.items(),
    )
    odin = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(odin_launch),
        launch_arguments={
            'bridge_config_file': LaunchConfiguration('bridge_config_file'),
            'mount_config_file': LaunchConfiguration('mount_config_file'),
            'odin_config_file': LaunchConfiguration('odin_config_file'),
            'pcd_path': LaunchConfiguration('pcd_path'),
        }.items(),
    )

    return LaunchDescription([
        bridge_config_arg,
        mount_config_arg,
        odin_config_arg,
        pcd_path_arg,
        debug_period_arg,
        runtime,
        odin,
    ])
