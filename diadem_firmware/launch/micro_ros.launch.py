from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        # SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=["serial", "--dev", "/dev/esp","-b", "921600"]),
        
        Node(
            package='diadem_firmware',
            executable='rbl_logger.pyc',
            name='RBL_LOGGER',
        )
    ])
