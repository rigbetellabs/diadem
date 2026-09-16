import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression,Command
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable,IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import launch_ros
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
  pkg_share = launch_ros.substitutions.FindPackageShare(package='diadem_description').find('diadem_description')
  navigation_dir = os.path.join(get_package_share_directory('diadem_navigation'), 'launch')
  rviz_launch_dir=os.path.join(get_package_share_directory('diadem_description'), 'launch')
  gazebo_launch_dir=os.path.join(get_package_share_directory('diadem_gazebo'), 'launch')
  ydlidar_launch_dir=os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch')
  # camera_launch_dir=os.path.join(get_package_share_directory('v4l2_camera'), 'launch')
  # cartographer_launch_dir=os.path.join(get_package_share_directory('diadem_slam'), 'launch')
  micro_ros_launch_dir=os.path.join(get_package_share_directory('diadem_firmware'), 'launch')
  prefix_address = get_package_share_directory('diadem_navigation') 
  default_model_path = os.path.join(pkg_share, 'urdf/diadem_sim.xacro')
  default_rviz_config_path = os.path.join(get_package_share_directory('diadem_description'), 'rviz/diadem_sensor_display.rviz')
    
  
  params_file_sim = os.path.join(prefix_address, 'config', 'nav2_params_simulation.yaml')
  params_file_robot = os.path.join(prefix_address, 'config', 'nav2_params.yaml')
  
  map_file=LaunchConfiguration('map')
  map_directory = os.path.join(get_package_share_directory(
        'diadem_navigation'), 'maps','test3.yaml')
  use_sim_time=LaunchConfiguration('use_sim_time')
  exploration=LaunchConfiguration('exploration')   
  
  rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters= [{'use_sim_time': use_sim_time}],

    )

  state_publisher_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'state_publisher.launch.py')),
            launch_arguments={'use_sim_time':use_sim_time,
                              'model': default_model_path}.items())

  gazebo_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'gazebo.launch.py')),
            condition=IfCondition(use_sim_time),
            launch_arguments={'use_sim_time':use_sim_time}.items())

  navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'navigation.launch.py')),
        launch_arguments={'params_file': params_file_robot}.items())
  
  ydlidar_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_launch_dir, 'ydlidar_launch.py')),
            condition=IfCondition(PythonExpression(['not ', use_sim_time])),
            launch_arguments={'use_sim_time':use_sim_time}.items())
  
  micro_ros_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(micro_ros_launch_dir, 'micro_ros.launch.py')),
            condition=IfCondition(PythonExpression(['not ', use_sim_time])),
            launch_arguments={'use_sim_time':use_sim_time}.items())
  
  camera_drive_node = Node(
        package='v4l2_camera',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        executable='v4l2_camera_node',
        name ='camera_publisher',
    )

  return LaunchDescription([

    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
    launch.actions.DeclareLaunchArgument(name='exploration', default_value='True',
                                            description='Flag to enable use_sim_time'),
    launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                          description='Absolute path to robot urdf file'),
    launch.actions.DeclareLaunchArgument(name='map',default_value=map_directory,
                                          description='Map to be used'),
    launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
    Node(
        package='nav2_map_server',
        condition=IfCondition(PythonExpression(['not ', exploration])),
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_file}
                    ]),
    Node(
        package='nav2_lifecycle_manager',
        condition=IfCondition(PythonExpression(['not ', exploration])),
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]),

    # rviz_node,
    state_publisher_launch_cmd,
    # # camera_drive_node,
    # gazebo_launch_cmd,
    ydlidar_launch_cmd,
    # navigation_launch_cmd,
    micro_ros_launch_cmd,

  ]
)