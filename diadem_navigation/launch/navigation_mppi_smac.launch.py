import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # Package directories
    pkg_navigation = get_package_share_directory('diadem_navigation')
    pkg_description = get_package_share_directory('diadem_description')
    pkg_gazebo = get_package_share_directory('diadem_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Simulation parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_name = LaunchConfiguration('world', default='nav2_test_world.sdf')
    headless = LaunchConfiguration('headless', default='False')

    # Paths to files
    model_xacro_path = os.path.join(pkg_description, 'urdf', 'diadem_sim.xacro')
    gz_bridge_yaml = os.path.join(pkg_navigation, 'config', 'gz_bridge_mppi.yaml')
    fast_lio_yaml = os.path.join(pkg_navigation, 'config', 'fast_lio_sim.yaml')
    ekf_yaml = os.path.join(pkg_navigation, 'config', 'ekf.yaml')
    navsat_yaml = os.path.join(pkg_navigation, 'config', 'navsat_transform.yaml')
    nav2_params_yaml = os.path.join(pkg_navigation, 'config', 'nav2_mppi_smac_params.yaml')

    # 1. Set environment variables for Gazebo Sim (Harmonic)
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_gazebo, 'worlds'),
            os.path.dirname(pkg_description)
        ]
    )

    # 2. Gazebo Simulator Command Generator (supporting Headless mode)
    def resolve_world_path(context, *args, **kwargs):
        w_name = world_name.perform(context)
        is_headless = headless.perform(context)
        
        if os.path.isabs(w_name):
            world_full_path = w_name
        else:
            world_full_path = os.path.join(pkg_gazebo, 'worlds', w_name)
        
        cmd = ['gz', 'sim', '-r']
        if is_headless.lower() in ['true', '1']:
            cmd.append('-s')
        cmd.extend([world_full_path, '--verbose', '1'])
        
        return [ExecuteProcess(
            cmd=cmd,
            output='screen'
        )]

    gz_sim = OpaqueFunction(function=resolve_world_path)

    # 3. Spawner node to create the robot model in simulation
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'default',
            '-name', 'diadem',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0.0'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 4. Robot State Publisher
    robot_description_content = ParameterValue(
        Command(['xacro ', model_xacro_path]),
        value_type=str
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }],
        output='screen'
    )

    # 5. Consolidated ROS-Gazebo Parameter Bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', ['config_file:=', gz_bridge_yaml]],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 6. PointCloud to LaserScan projection
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
            'angle_increment': 0.0087, # ~0.5 deg resolution
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 70.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # 7. Local EKF: publishes odom -> base_link TF (without GPS)
    local_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='local_ekf_filter_node',
        parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odometry/local')],
        output='screen'
    )

    # Global EKF: publishes map -> odom TF (fuses GPS)
    global_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='global_ekf_filter_node',
        parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odometry/global')],
        output='screen'
    )

    # Static transforms to map FAST-LIO2 hardcoded frame names (body, camera_init) to base_link and odom
    static_base_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_body_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'body'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    odom_relay = Node(
        package='diadem_navigation',
        executable='odom_frame_relay.py',
        name='odom_frame_relay',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 8. FAST-LIO2 Mapping Node
    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio_mapping',
        parameters=[
            fast_lio_yaml,
            {'use_sim_time': use_sim_time, 'publish_tf': False}
        ],
        remappings=[
            ('/cloud_registered', '/fast_lio/cloud_registered'),
            ('/Odometry', '/fast_lio/odometry')
        ],
        output='screen'
    )

    # 9. robot_localization - NavSat Transform Node
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        parameters=[navsat_yaml, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu/data', '/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odometry/global'),
            ('odometry/gps', '/odometry/gps')
        ],
        output='screen'
    )

    # 11. Nav2 Navigation Stack
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

    return LaunchDescription([
        set_gz_resource_path,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='nav2_test_world.sdf',
            description='World file name'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='Whether to run Gazebo headless (server only)'
        ),
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        gz_bridge,
        pc2scan,
        static_base_body,
        odom_relay,
        fast_lio,
        local_ekf,
        global_ekf,
        navsat_transform,
        nav2_bringup
    ])
