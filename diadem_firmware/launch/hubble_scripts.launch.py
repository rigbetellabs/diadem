import launch
import launch_ros

def generate_launch_description():

    network_data_streamer_node = launch_ros.actions.Node(
        package='diadem_firmware',
        executable='network_status_publisher',
        name='network_data',
    )
    goal_status_publisher_node = launch_ros.actions.Node(
        package='diadem_firmware',
        executable='goal_status_publisher',
        name='goal_status_streamer',
    )
    return launch.LaunchDescription([
        network_data_streamer_node,
        goal_status_publisher_node
    ])
    