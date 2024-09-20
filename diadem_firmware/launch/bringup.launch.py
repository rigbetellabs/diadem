
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    firmware_dir = os.path.join(get_package_share_directory('diadem_firmware'), 'launch')
    realsense_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'launch')
    realsense = LaunchConfiguration('realsense')
    stats_dir = os.path.join(get_package_share_directory('rbl_robot_stats'), 'launch')
   
    camera_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_launch_dir, 'rs_launch.py')),
        condition=IfCondition(realsense),
        launch_arguments={
            'rgb_camera.color_profile': '640,360,6',
        }.items())
    
    micro_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(firmware_dir,'micro_ros.launch.py')))
    
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(firmware_dir,'mavros_launch.py')))

    hubble_scripts_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(firmware_dir,'hubble_scripts.launch.py')))
    
    stats_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stats_dir, 'robot_stats.launch.py')))
    
    # Create and return the launch description with both launch files
    return LaunchDescription([
        DeclareLaunchArgument(name='realsense', default_value='True',
                                              description='To launch realsense camera'),

        micro_ros_launch,
        mavros_launch,
        hubble_scripts_launch,
        # stats_node,
        camera_launch_cmd
    ])

if __name__ == '__main__':
    generate_launch_description()