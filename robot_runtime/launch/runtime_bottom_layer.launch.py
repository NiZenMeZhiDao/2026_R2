from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'debug_period_sec',
            default_value='0.5',
            description='Runtime debug topic publish period in seconds; <=0 disables it.',
        ),
        Node(
            package='ares_usb',
            executable='usb_bridge_node',
            name='usb_bridge_node',
            output='screen',
        ),
        Node(
            package='multi_serial_sensor',
            executable='multi_serial_node',
            name='multi_serial_node',
            output='screen',
        ),
        Node(
            package='robot_runtime',
            executable='runtime_node',
            name='robot_runtime_node',
            output='screen',
            parameters=[{
                'debug_period_sec': LaunchConfiguration('debug_period_sec'),
            }],
        ),
    ])
