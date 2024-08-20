import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def setup_realsense_config():
    config_file_path = os.path.join(
        get_package_share_directory("px4_odom"),
        'config',
        'rs_stereo.yaml'
    )

    with open(config_file_path, 'r') as file:
        config_params = yaml.safe_load(file)

    return config_params

def generate_launch_description():
    config_params = setup_realsense_config()
    rs_launch_arguments = [(key, str(value)) for key, value in config_params.items()]

    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
        launch_arguments=rs_launch_arguments
    )

    return LaunchDescription([
        rs_launch,
    ])

