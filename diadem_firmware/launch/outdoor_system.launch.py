#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch micro_ros_agent first
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/esp', '-b', '921600'],
            output='screen'
        ),
        
        # Launch mavros after 3 seconds delay
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='mavros',
                    executable='mavros_node',
                    parameters=[
                        {'gcs_url': 'udp://@192.168.0.103:14550'},
                        {'fcu_url': '/dev/pixhwak:921600'}
                    ],
                    output='screen'
                )
            ]
        ),
        
        # Launch pixhawk_to_cmd.py after 6 seconds delay (3 seconds after mavros)
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='diadem_firmware',
                    executable='pixhawk_to_cmd.py',
                    name='pixhawk_to_cmd',
                    output='screen'
                )
            ]
        ),
                TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_tcp_endpoint',
                    executable='default_server_endpoint',
                    parameters=[
                        {'ROS_IP': '192.168.0.101'}
                    ],
                    output='screen'
                )
            ]
        ),
    ])
