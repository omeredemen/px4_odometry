import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("px4_odom"),
        'config',
        'config.yaml'
        )
    
    return LaunchDescription([
        Node(
            package='px4_odom',
            executable='px4_odom_node',
            parameters=[config]
        )
    ])