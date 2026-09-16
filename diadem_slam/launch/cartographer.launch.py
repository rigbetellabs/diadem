import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
  prefix_address = get_package_share_directory('diadem_slam') 
  config_directory = os.path.join(prefix_address, 'config')
  slam_config = 'slam.lua'
  res = LaunchConfiguration('resolution', default='0.05')
  publish_period = LaunchConfiguration('publish_period_sec', default='1.0')
  use_sim_time=LaunchConfiguration('use_sim_time')
  exploration=LaunchConfiguration('exploration')  # slam in exploration MODE


  return LaunchDescription([

    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
    launch.actions.DeclareLaunchArgument(name='exploration', default_value='True',
                                            description='Flag to enable use_sim_time'),
    DeclareLaunchArgument(
      'resolution',
      default_value=res,
      description='configure the resolution'
    ),

    DeclareLaunchArgument(
      'publish_period_sec',
      default_value=publish_period,
      description='publish period in seconds'
    ),

    ################### cartographer_ros_node ###################
    DeclareLaunchArgument(
      'configuration_directory',
      default_value=config_directory,
      description='path to the .lua files'
    ),
    DeclareLaunchArgument(
      'slam_configuration_basename',
      default_value=slam_config,
      description='name of .lua file to be used'
    ),
    DeclareLaunchArgument(
      'localization_configuration_basename',
      default_value=slam_config,
      description='name of .lua file to be used'
    ),
    DeclareLaunchArgument(
      'scan_topic',
      default_value=PythonExpression(["'/scan' if '", use_sim_time, "' == 'True' or '", use_sim_time, "' == 'true' else '/scan_filtered'"]),
      description='LaserScan topic to subscribe to'
    ),

    Node(
      package='cartographer_ros',
      condition= IfCondition(exploration),
      executable='cartographer_node',
      name='as21_cartographer_node',
      arguments=[
        '-configuration_directory', config_directory,
        '-configuration_basename', slam_config
      ],
      parameters= [{'use_sim_time':use_sim_time}],
      remappings=[
        ('odom', 'odom'),
        ('imu', '/imu/data'),
        ('scan', LaunchConfiguration('scan_topic'))
      ],
      output='screen'
    ),
    Node(
      package='cartographer_ros',
      condition=IfCondition(PythonExpression(['not ', exploration])),
      executable='cartographer_node',
      name='as21_cartographer_node',
      arguments=[
        '-configuration_directory', config_directory,
        '-configuration_basename', slam_config
      ],
      parameters= [{'use_sim_time':use_sim_time}],
      remappings=[
        ('odom', 'odom'),
        ('imu', '/imu/data'),
        ('scan', LaunchConfiguration('scan_topic'))
      ],
      output='screen'
    ),

    Node(
      package='cartographer_ros',
      condition= IfCondition(exploration),
      executable='cartographer_occupancy_grid_node',
      name='cartographer_occupancy_grid_node',
      arguments=[
        '-resolution', res,
        '-publish_period_sec', publish_period
      ],
      parameters=[{'use_sim_time': use_sim_time}]
    ),   

  ]
)
