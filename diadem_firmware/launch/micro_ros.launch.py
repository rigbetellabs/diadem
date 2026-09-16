from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/esp',
        description='Serial port for micro-ROS agent'
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='921600',
        description='Baudrate for micro-ROS agent'
    )

    micro_ros_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', LaunchConfiguration('port'), '-b', LaunchConfiguration('baudrate')],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        micro_ros_node,
    ])
