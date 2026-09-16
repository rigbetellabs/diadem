import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/imu',
        description='Serial port for BNO055 IMU'
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate for BNO055 IMU'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for IMU message'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation clock if true'
    )

    imu_node = Node(
        package='diadem_firmware',
        executable='imu_read_pub.py',
        name='bno055_imu_publisher',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        frame_id_arg,
        use_sim_time_arg,
        imu_node,
    ])
