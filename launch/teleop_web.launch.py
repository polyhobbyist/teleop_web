from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cert_launch_arg = DeclareLaunchArgument(
        'certfile',
        default_value=''
    )

    key_launch_arg = DeclareLaunchArgument(
        'keyfile',
        default_value=''
    )

    port_launch_arg = DeclareLaunchArgument(
        'port',
        default_value='8088'
    )

    address_launch_arg = DeclareLaunchArgument(
        'address',
        default_value=''
    )

    return LaunchDescription([
        cert_launch_arg,
        key_launch_arg,
        port_launch_arg,
        address_launch_arg,
        Node(
            package='teleop_web',
            executable='teleop_web',
            name='teleop_web',
            output='screen',
            parameters=[{
                'certfile': LaunchConfiguration('certfile'),
                'keyfile': LaunchConfiguration('keyfile'),
                'port': LaunchConfiguration('port'),
                'address': LaunchConfiguration('address'),
           }]
        ),
    ])
