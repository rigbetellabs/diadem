import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    realsense_launch_dir=os.path.join(get_package_share_directory('realsense2_camera'), 'launch')
    
    camera_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_launch_dir, 'rs_launch.py')
        ),
        launch_arguments={
            'pointcloud.enable': 'True',  
        }.items()
    )
    
    image_compressed = Node(
        package='image_transport',
        executable='republish',
        name='cam_compressed',
        output='screen',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/camera/color/image_raw'),
            ('out', '/image')
        ],
        parameters=[
            {
                'in_transport': 'raw',
                'out_transport': 'compressed',
                'input_transport': 'raw',
                'output_transport': 'compressed'
            }
        ]
    )
    
    
    return LaunchDescription([
        camera_launch_cmd,
        image_compressed
        
        

    ]
)
