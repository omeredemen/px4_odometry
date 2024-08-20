import os
import yaml

from ament_index_python import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def setup_realsense_config():
    config_file_path = os.path.join(
        get_package_share_directory('px4_odom'),
        'config',
        'rs_stereo.yaml'
    )

    with open(config_file_path, 'r') as file:
        config_params = yaml.safe_load(file)

    return config_params

def generate_launch_description():
    rviz_enable = DeclareLaunchArgument(name='ov_enable', default_value='true')
    ov_config_path = DeclareLaunchArgument(name='ov_config_path', default_value='')
    rviz_enable = DeclareLaunchArgument(name='rviz_enable', default_value='true')
    realsense_package = get_package_share_directory('realsense2_camera')
    openvins_package = get_package_share_directory('ov_msckf')

    config_params = setup_realsense_config()
    rs_launch_arguments = [(key, str(value)) for key, value in config_params.items()]
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            realsense_package, 'launch', 'rs_launch.py')
        ]),
        launch_arguments=rs_launch_arguments
    )

    openvins_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            openvins_package, 'launch', 'subscribe.launch.py')
        ]),
        launch_arguments={'rviz_enable': LaunchConfiguration('rviz_enable'),
                          'config_path': LaunchConfiguration('ov_config_path')}.items(),
        condition=IfCondition(LaunchConfiguration('ov_enable'))
    )
    return LaunchDescription([
        ov_config_path,
        rviz_enable,
        rs_launch,
        openvins_launch
    ])