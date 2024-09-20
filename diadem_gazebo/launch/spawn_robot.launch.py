import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros

def generate_launch_description():
  pkg_share = launch_ros.substitutions.FindPackageShare(package='diadem_description').find('diadem_description')
  state_publisher_launch_dir=os.path.join(get_package_share_directory('diadem_description'), 'launch')
  gazebo_launch_dir=os.path.join(get_package_share_directory('diadem_gazebo'), 'launch')
  default_model_path = os.path.join(pkg_share, 'urdf/diadem_sim.xacro')


  state_publisher_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(state_publisher_launch_dir, 'state_publisher.launch.py')),
        launch_arguments={'model': default_model_path}.items())

  gazebo_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'gazebo.launch.py')),)

  return LaunchDescription([

    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                          description='Absolute path to robot urdf file'),

    state_publisher_launch_cmd,
    gazebo_launch_cmd,

  ]
)

