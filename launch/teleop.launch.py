from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    websocket_launch_arg = DeclareLaunchArgument(
        'websocket',
        default_value='9090'
    )

    address_launch_arg = DeclareLaunchArgument(
        'address',
        default_value=''
    )

    return LaunchDescription([
        cert_launch_arg,
        key_launch_arg,
        port_launch_arg,
        websocket_launch_arg,
        address_launch_arg,
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'certfile': LaunchConfiguration('certfile'),
                'keyfile': LaunchConfiguration('keyfile'),
                'port': LaunchConfiguration('websocket'),
                'address': LaunchConfiguration('address'),
           }]
        ),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare("teleop_twist_joy"), '/launch', '/teleop-launch.py'])
        ),

    ])
