#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('diadem_odom')
    config_file = os.path.join(pkg_share, 'config', 'ekf_params.yaml')
    
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file]
        ),
        # Node(
        #         package="robot_localization",
        #         executable="ekf_node",
        #         name="ekf_filter_node_map",
        #         output="screen",
        #         parameters=[config_file],
        #         remappings=[("odometry/filtered", "odometry/global")],
      
        #     ),

        # Node(
        #     package='tf2_ros', 
        #     executable='static_transform_publisher', 
        #     output='screen',
        #     # Rotate 90 degrees around Z axis (0, 0, 1.571)
        #     arguments=['0', '0', '0', '-1.57', '0', '0', 'map', 'map_rotated']
        # )
    ])


