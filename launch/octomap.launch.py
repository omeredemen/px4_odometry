import os
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    octomap_package = get_package_share_directory('octomap_server2')

    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='global'),
        DeclareLaunchArgument('base_frame_id', default_value='base_link'),
        DeclareLaunchArgument('pcl_topic', default_value='/d456/d456/depth/color/points'),
        DeclareLaunchArgument('resolution', default_value='0.15'),
        DeclareLaunchArgument('height_map', default_value='False'),
        DeclareLaunchArgument('colored_map', default_value='False'),
        DeclareLaunchArgument('compress_map', default_value='False'),
        DeclareLaunchArgument('incremental_2D_projection', default_value='False'),
        DeclareLaunchArgument('sensor_model/max_range', default_value='5.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                octomap_package, 'launch', 'octomap_server_launch.py')
            ]),
            launch_arguments={'frame_id': LaunchConfiguration('frame_id'),
                            'base_frame_id': LaunchConfiguration('base_frame_id'),
                            'input_cloud_topic': LaunchConfiguration('pcl_topic'),
                            'resolution': LaunchConfiguration('resolution'),
                            'height_map': LaunchConfiguration('height_map'),
                            'colored_map': LaunchConfiguration('colored_map'),
                            'compress_map': LaunchConfiguration('compress_map'),
                            'incremental_2D_projection': LaunchConfiguration('incremental_2D_projection'),
                            'sensor_model/max_range': LaunchConfiguration('sensor_model/max_range')}.items()
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0','0','0','0.5','-0.5','0.5','0.5', 'imu', LaunchConfiguration('base_frame_id')]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0','0','0','0','0','0','1.0', LaunchConfiguration('base_frame_id'), 'd456_link']
        )
    ])