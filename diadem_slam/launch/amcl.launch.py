import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    slam_dir = get_package_share_directory('diadem_slam')
    config_directory = os.path.join(slam_dir, 'config')
    config_basename = 'amcl.lua'

    use_sim_time = LaunchConfiguration('use_sim_time')

    scan_topic = PythonExpression([
        "'/scan' if '", use_sim_time, "' in ['True', 'true', '1'] else '/scan_filtered'"
    ])

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='False',
            description='Flag to enable use_sim_time'
        ),
        DeclareLaunchArgument(
            name='configuration_directory',
            default_value=config_directory,
            description='Path to the .lua config directory'
        ),
        DeclareLaunchArgument(
            name='configuration_basename',
            default_value=config_basename,
            description='Name of the .lua configuration file'
        ),
        DeclareLaunchArgument(
            name='scan_topic',
            default_value=scan_topic,
            description='LaserScan topic to subscribe to'
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            arguments=[
                '-configuration_directory', LaunchConfiguration('configuration_directory'),
                '-configuration_basename', LaunchConfiguration('configuration_basename')
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('odom', 'odom'),
                ('imu', '/imu/data'),
                ('scan', LaunchConfiguration('scan_topic'))
            ],
            output='screen'
        )
    ])
