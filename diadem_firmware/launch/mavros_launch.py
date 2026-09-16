from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://@192.168.0.103:14550',
        description='GCS URL for MAVROS'
    )
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/pixhwak:921600',
        description='FCU URL for Pixhawk connection'
    )

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[{
            'gcs_url': LaunchConfiguration('gcs_url'),
            'fcu_url': LaunchConfiguration('fcu_url')
        }],
        respawn=True,
        respawn_delay=2.0
    )

    pixhawk_to_cmd_node = Node(
        package='diadem_firmware',
        executable='pixhawk_to_cmd.py',
        name='pixhawk_to_cmd',
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        gcs_url_arg,
        fcu_url_arg,
        mavros_node,
        pixhawk_to_cmd_node,
    ])
