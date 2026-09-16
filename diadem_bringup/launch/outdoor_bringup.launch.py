import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    firmware_launch_dir = os.path.join(
        get_package_share_directory('diadem_firmware'), 'launch'
    )

    esp_port_arg = DeclareLaunchArgument(
        'esp_port',
        default_value='/dev/esp',
        description='Serial port for micro-ROS agent'
    )
    esp_baud_arg = DeclareLaunchArgument(
        'esp_baudrate',
        default_value='921600',
        description='Baudrate for micro-ROS agent'
    )
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/pixhawk:921600',
        description='FCU URL for Pixhawk / MAVROS connection'
    )
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://@192.168.0.103:14550',
        description='GCS URL for MAVROS'
    )

    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', LaunchConfiguration('esp_port'), '-b', LaunchConfiguration('esp_baudrate')],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    mavros_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(firmware_launch_dir, 'mavros_launch.py')
        ),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
        }.items()
    )

    hubble_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(firmware_launch_dir, 'hubble_scripts.launch.py')
        )
    )

    return LaunchDescription([
        esp_port_arg,
        esp_baud_arg,
        fcu_url_arg,
        gcs_url_arg,
        micro_ros_agent_node,
        mavros_launch_cmd,
        hubble_launch_cmd,
    ])
