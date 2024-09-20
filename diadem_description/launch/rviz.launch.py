import launch
import os
from launch.substitutions import LaunchConfiguration
import launch_ros
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='diadem_description').find('diadem_description')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/display.rviz')
    rvizconfig=LaunchConfiguration('rvizconfig')   

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        
        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizconfig],
        )

    ])
    