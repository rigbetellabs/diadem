import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                            IncludeLaunchDescription, TimerAction,
                            ExecuteProcess)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
import launch_ros
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    description_package_path = get_package_share_directory('diadem_description')
    navigation_dir      = os.path.join(get_package_share_directory('diadem_navigation'), 'launch')
    rviz_launch_dir     = os.path.join(description_package_path, 'launch')
    gazebo_launch_dir   = os.path.join(get_package_share_directory('diadem_gazebo'), 'launch')
    ydlidar_launch_dir  = os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch')
    cartographer_launch_dir = os.path.join(get_package_share_directory('diadem_slam'), 'launch')
    micro_ros_launch_dir    = os.path.join(get_package_share_directory('diadem_firmware'), 'launch')
    ekf_launch_dir          = os.path.join(get_package_share_directory('diadem_odom'), 'launch')
    prefix_address = get_package_share_directory('diadem_navigation')

    default_rviz_config_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz', 'nav2_default_view.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    exploration  = LaunchConfiguration('exploration')
    realsense    = LaunchConfiguration('realsense')

    params_file_robot = os.path.join(prefix_address, 'config', 'nav2_params.yaml')
    params_file_sim   = os.path.join(prefix_address, 'config', 'nav2_params_sim.yaml')

    nav2_params_file = PythonExpression([
        "'" + params_file_sim + "' if '", use_sim_time, "' in ['True', 'true', '1'] else '" + params_file_robot + "'"
    ])

    map_file      = LaunchConfiguration('map')
    map_directory = os.path.join(
        get_package_share_directory('diadem_navigation'), 'maps', 'nav2_test_map.yaml')

    x_pose   = LaunchConfiguration('x_pose',   default='0.0')
    y_pose   = LaunchConfiguration('y_pose',   default='0.0')
    z_pose   = LaunchConfiguration('z_pose',   default='0.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0')

    # AMCL (Localization)
    amcl_node = Node(
        package='nav2_amcl',
        condition=IfCondition(PythonExpression(['not ', exploration])),
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file,
                    {'use_sim_time': use_sim_time}]
    )

    # RViz with Nav2 default config
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # State publisher (robot_state_publisher + static TFs)
    state_publisher_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'state_publisher.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'model': os.path.join(description_package_path, 'urdf', 'diadem_sim.xacro')
        }.items())

    # Simulation: Ignition Fortress (World)
    gazebo_world_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'gazebo.launch.py')),
        condition=IfCondition(use_sim_time),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'headless': LaunchConfiguration('headless'),
            'world': LaunchConfiguration('world'),
        }.items())

    # Simulation: Robot Spawner
    spawn_robot_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'spawn_robot.launch.py')),
        condition=IfCondition(use_sim_time),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose':   x_pose,
            'y_pose':   y_pose,
            'z_pose':   z_pose,
            'yaw_pose': yaw_pose,
        }.items())

    # Real robot: YDLidar driver
    ydlidar_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_launch_dir, 'ydlidar_launch.py')),
        condition=UnlessCondition(use_sim_time),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    # Real robot: micro-ROS agent
    micro_ros_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(micro_ros_launch_dir, 'micro_ros.launch.py')),
        condition=UnlessCondition(use_sim_time),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    # Real robot: BNO055 IMU driver (launches only when use_sim_time is False)
    imu_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(micro_ros_launch_dir, 'imu.launch.py')),
        condition=UnlessCondition(use_sim_time),
        launch_arguments={
            'serial_port': '/dev/imu',
            'baudrate': '115200',
            'use_sim_time': use_sim_time,
        }.items())

    # Hubble scripts (network status & goal status)
    hubble_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(micro_ros_launch_dir, 'hubble_scripts.launch.py'))
    )

    # Real robot: RealSense camera (D435i)
    realsense_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(micro_ros_launch_dir, 'realsense_d435i.launch.py')),
        condition=IfCondition(realsense)
    )

    # Real robot: camera
    # camera_drive_node = Node(
    #     package='v4l2_camera',
    #     condition=IfCondition(PythonExpression(['not ', use_sim_time])),
    #     executable='v4l2_camera_node',
    #     name='camera_publisher',
    # )

    # SLAM (Cartographer)
    cartographer_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cartographer_launch_dir, 'cartographer.launch.py')),
        condition=IfCondition(exploration),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'exploration': exploration,
        }.items())

    # EKF (Odometry filtering) - suppressed, using Cartographer local scan matching instead
    # ekf_launch_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(ekf_launch_dir, 'ekf.launch.py')),
    #     condition=IfCondition(PythonExpression(['not ', exploration])),
    #     launch_arguments={'use_sim_time': use_sim_time}.items())

    # Cartographer local scan matching for AMCL (provides odom -> base_link TF)
    amcl_carto_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cartographer_launch_dir, 'amcl.launch.py')),
        condition=IfCondition(PythonExpression(['not ', exploration])),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    # Navigation (Nav2)
    navigation_launch_cmd = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(navigation_dir, 'navigation.launch.py')),
                launch_arguments={
                    'params_file':  nav2_params_file,
                    'use_sim_time': use_sim_time,
                    'exploration':  exploration,
                }.items())
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            name='use_sim_time', default_value='False',
            description='Set True for Ignition sim, False for real robot'),
        DeclareLaunchArgument(
            name='realsense', default_value='False',
            description='Set True to launch Intel RealSense D435i camera, False otherwise'),
        DeclareLaunchArgument(
            name='exploration', default_value='True',
            description='Enable SLAM exploration (True) or navigate with pre-built map (False)'),
        DeclareLaunchArgument(
            name='model',
            default_value=os.path.join(description_package_path, 'urdf', 'diadem_sim.xacro'),
            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(
            name='world',
            default_value='nav2_test_world.sdf',
            description='Sim world filename (relative to diadem_gazebo/worlds/) or absolute path. '
                        'Options: nav2_test_world.sdf | open_field.sdf | room2.sdf'),
        DeclareLaunchArgument(
            name='headless', default_value='False',
            description='Whether to run Gazebo headless (server only, no GUI client)'),
        DeclareLaunchArgument(
            name='map', default_value=map_directory,
            description='Path to map yaml (used when exploration=False)'),
        DeclareLaunchArgument(
            name='rvizconfig', default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(
            name='x_pose', default_value='0.0',
            description='Simulation spawn x position'),
        DeclareLaunchArgument(
            name='y_pose', default_value='0.0',
            description='Simulation spawn y position'),
        DeclareLaunchArgument(
            name='z_pose', default_value='0.1',
            description='Simulation spawn z position'),
        DeclareLaunchArgument(
            name='yaw_pose', default_value='0.0',
            description='Simulation spawn yaw orientation'),

        # Map server + lifecycle manager (only when loading a pre-built map)
        Node(
            package='nav2_map_server',
            condition=IfCondition(PythonExpression(['not ', exploration])),
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename': map_file}]),
        Node(
            package='nav2_lifecycle_manager',
            condition=IfCondition(PythonExpression(['not ', exploration])),
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]),

        state_publisher_launch_cmd,  
        #rviz_node,
        gazebo_world_launch_cmd,     
        spawn_robot_launch_cmd,      
        ydlidar_launch_cmd,
        micro_ros_launch_cmd,
        imu_launch_cmd,
        hubble_launch_cmd,
        realsense_launch_cmd,
        # ekf_launch_cmd,
        amcl_carto_launch_cmd,
        cartographer_launch_cmd,      
        amcl_node,                    
        navigation_launch_cmd,        
    ])
