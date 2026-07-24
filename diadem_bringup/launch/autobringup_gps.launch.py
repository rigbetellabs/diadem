import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    pkg_bringup = get_package_share_directory('diadem_bringup')
    pkg_navigation = get_package_share_directory('diadem_navigation')
    pkg_description = get_package_share_directory('diadem_description')
    pkg_gazebo = get_package_share_directory('diadem_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    rvizconfig = LaunchConfiguration('rvizconfig')
    rviz_opt = LaunchConfiguration('rviz')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')

    model_sim_xacro = os.path.join(pkg_description, 'urdf', 'diadem_sim.xacro')
    model_real_xacro = os.path.join(pkg_description, 'urdf', 'diadem.xacro')
    gz_bridge_yaml = os.path.join(pkg_navigation, 'config', 'gz_bridge_mppi.yaml')
    fast_lio_sim_yaml = os.path.join(pkg_navigation, 'config', 'fast_lio_sim.yaml')
    fast_lio_hw_yaml = os.path.join(pkg_navigation, 'config', 'fast_lio_hw.yaml')
    ekf_yaml = os.path.join(pkg_navigation, 'config', 'ekf.yaml')
    navsat_yaml = os.path.join(pkg_navigation, 'config', 'navsat_transform.yaml')
    nav2_params_yaml = os.path.join(pkg_navigation, 'config', 'nav2_mppi_smac_params.yaml')
    default_rviz_config = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')

    set_gz_version = SetEnvironmentVariable(
        name='GZ_VERSION',
        value='harmonic'
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_gazebo, 'worlds'),
            os.path.dirname(pkg_gazebo),
            os.path.dirname(pkg_description)
        ])
    )

    def resolve_world_path(context, *args, **kwargs):
        import shutil
        sim_time = use_sim_time.perform(context)
        if sim_time.lower() in ['false', '0']:
            return []

        w_name = world_name.perform(context)
        is_headless = headless.perform(context)
        
        if os.path.isabs(w_name):
            world_full_path = w_name
        else:
            world_full_path = os.path.join(pkg_gazebo, 'worlds', w_name)
        
        gz_path = shutil.which('gz')
        ign_path = shutil.which('ign')
        if gz_path:
            cmd = [gz_path, 'sim', '-r']
        elif ign_path:
            cmd = [ign_path, 'gazebo', '-r']
        else:
            cmd = ['gz', 'sim', '-r']

        if is_headless.lower() in ['true', '1']:
            cmd.append('-s')
        cmd.extend([world_full_path, '--verbose', '1'])
        
        return [ExecuteProcess(
            cmd=cmd,
            output='screen'
        )]

    gz_sim = OpaqueFunction(function=resolve_world_path)

    # Spawner node
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        condition=IfCondition(use_sim_time),
        arguments=[
            '-name', 'diadem',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Robot State Publisher - Simulation
    robot_description_sim = ParameterValue(
        Command(['xacro ', model_sim_xacro]),
        value_type=str
    )
    rsp_sim = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_sim',
        condition=IfCondition(use_sim_time),
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_sim
        }],
        output='screen'
    )

    # ROS-Gazebo Parameter Bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        condition=IfCondition(use_sim_time),
        arguments=['--ros-args', '-p', ['config_file:=', gz_bridge_yaml]],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    robot_description_real = ParameterValue(
        Command(['xacro ', model_real_xacro]),
        value_type=str
    )
    rsp_real = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_real',
        condition=UnlessCondition(use_sim_time),
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_real
        }],
        output='screen'
    )

    # ESP32 Serial Driver Bridge (via micro-ROS agent)
    esp_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        condition=UnlessCondition(use_sim_time),
        arguments=['serial', '--dev', '/dev/esp', '-b', '921600'],
        output='screen'
    )

    # Pixhawk MAVROS GPS receiver
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros_node',
        condition=UnlessCondition(use_sim_time),
        parameters=[
            {'gcs_url': 'udp-b://:14550@'},
            {'fcu_url': '/dev/px4:921600'}
        ],
        remappings=[
            ('/mavros/global_position/raw/fix', '/gps/fix')
        ],
        output='screen'
    )

    # Livox Mid-360 LiDAR driver
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_ros_driver2_node',
        condition=UnlessCondition(use_sim_time),
        parameters=[{
            'livox_class': 0,
            'device_type': 0,
            'publish_freq': 10.0,
            'data_src': 0,
            'xfer_format': 1,
            'multi_topic': 0,
            'user_config_path': '',
            'cmdline_input_bd_code': '000000000000001'
        }],
        remappings=[
            ('/livox/lidar', '/points_raw')
        ],
        output='screen'
    )

    odom_relay = Node(
        package='diadem_navigation',
        executable='odom_frame_relay.py',
        name='odom_frame_relay',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': False,
            'approx_sync': True,
            'approx_sync_max_interval': 0.05,
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'use_sim_time': use_sim_time,
            'Odom/MinInliers': '10',
            'Vis/MinInliers': '10',
            'publish_null_when_lost': True,
            'guess_frame_id': 'odom',
            'Odom/ResetCountdown': '1',
            'Vis/FeatureType': '8',
            'Vis/CorGuessWinSize': '100',
            'always_process_most_recent_frame': False
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('depth/image', '/camera/depth'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('odom', '/vio/odom')
        ],
        output='screen'
    )

    # 1. Local EKF: publishes odom -> base_link TF (without GPS)
    local_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='local_ekf_filter_node',
        parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odometry/local')],
        output='screen'
    )

    # 2. Global EKF: publishes map -> odom TF (fuses GPS)
    global_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='global_ekf_filter_node',
        parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odometry/global')],
        output='screen'
    )

    # NavSat Transform: converts GPS fixes (/gps/fix) to ENU odometry (/odometry/gps)
    # Needs /odometry/global from EKF to anchor GPS to the local frame
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        parameters=[navsat_yaml, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu/data',           '/imu/data'),
            ('gps/fix',            '/gps/fix'),
            ('odometry/filtered',  '/odometry/global'),
            ('odometry/gps',       '/odometry/gps')
        ],
        output='screen'
    )

    # PointCloud to LaserScan projection
    pc2scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/points_raw'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 2.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 100.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # FAST-LIO2 Mapping (Simulation configuration)
    fast_lio_sim = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio_mapping',
        condition=IfCondition(use_sim_time),
        parameters=[
            fast_lio_sim_yaml,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/cloud_registered', '/fast_lio/cloud_registered'),
            ('/Odometry', '/fast_lio/odometry')
        ],
        output='screen'
    )

    # FAST-LIO2 Mapping (Hardware configuration)
    fast_lio_hw = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio_mapping',
        condition=UnlessCondition(use_sim_time),
        parameters=[
            fast_lio_hw_yaml,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/cloud_registered', '/fast_lio/cloud_registered'),
            ('/Odometry', '/fast_lio/odometry')
        ],
        output='screen'
    )

    # Nav2 Navigation Stack
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_yaml,
            'autostart': 'True'
        }.items()
    )

    # RViz visualization (optional, disabled by default for headless runs)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(rviz_opt),
        output='screen',
        arguments=['-d', rvizconfig],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        set_gz_version,
        set_gz_resource_path,
        DeclareLaunchArgument(
            name='use_sim_time', default_value='True',
            description='Set True for Gazebo simulation, False for real hardware'),
        DeclareLaunchArgument(
            name='world', default_value='tugbot_depot.sdf',
            description='Simulation world filename'),
        DeclareLaunchArgument(
            name='headless', default_value='False',
            description='Run Gazebo headless (server only, no GUI)'),
        DeclareLaunchArgument(
            name='rviz', default_value='False',
            description='Whether to launch RViz2 visualization'),
        DeclareLaunchArgument(
            name='rvizconfig', default_value=default_rviz_config,
            description='Absolute path to RViz config file'),
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

        gz_sim,
        spawn_robot,
        rsp_sim,
        gz_bridge,
        fast_lio_sim,
        rsp_real,
        esp_agent,
        mavros_node,
        livox_driver,
        fast_lio_hw,

        rgbd_odometry,
        odom_relay,
        local_ekf,
        global_ekf,
        navsat_transform,
        pc2scan,
        nav2_bringup,
        rviz_node
    ])
