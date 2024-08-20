import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def setup_realsense():
    config_file_path = os.path.join(
        get_package_share_directory("px4_odom"),
        'config',
        'rs_stereo.yaml'
    )

    with open(config_file_path, 'r') as file:
        config_params = yaml.safe_load(file)

    return config_params

def generate_launch_description():
    config_params = setup_realsense()
    camera_name = config_params.get('camera_name')
    camera_namespace = config_params.get('camera_namespace')
    topic_prefix = camera_namespace + '/' +camera_name
    ## Topics
    declare_args = [
        DeclareLaunchArgument(name='rviz_enable', default_value='true'),
        DeclareLaunchArgument(name='slam_enable', default_value='true'),
        DeclareLaunchArgument(name='rs_enable', default_value='true'),

        DeclareLaunchArgument(name='use_mag', default_value='false', description='Use magnetometer while computing rotation'),
        DeclareLaunchArgument(name='world_frame', default_value='enu', description='Default world frame'),
        DeclareLaunchArgument(name='publish_tf_madgwick', default_value='false', description='Enable publishing tf'),

        DeclareLaunchArgument(name='frame_id',default_value= camera_name+'_link', description= 'frame id'),
        DeclareLaunchArgument(name='subscribe_stereo', default_value= 'true', description= 'subscribe stereo cameras for odometry'),
        DeclareLaunchArgument(name='subscribe_odom_info', default_value= 'true', description= 'subscribe odom information'),
        DeclareLaunchArgument(name='wait_imu_to_init', default_value= 'true', description= 'wait imu to initialize before start start odom and slam'),
    ]

    rtabmap_remap_topics =[
        ('imu', '/imu/data'),
        ('left/image_rect',  topic_prefix + '/infra1/image_rect_raw'),
        ('left/camera_info', topic_prefix +'/infra1/camera_info'),
        ('right/image_rect', topic_prefix + '/infra2/image_rect_raw'),
        ('right/camera_info', topic_prefix + '/infra2/camera_info')
    ]

    rtabmap_parameters=[
        {'frame_id': LaunchConfiguration('frame_id')},
        {'subscribe_stereo': LaunchConfiguration('subscribe_stereo')},
        {'subscribe_odom_info': LaunchConfiguration('subscribe_odom_info')},
        {'wait_imu_to_init': LaunchConfiguration('wait_imu_to_init')}
    ]

    ## Nodes
    rs_launch_arguments = [(key, str(value)) for key, value in config_params.items()]
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
        launch_arguments=rs_launch_arguments,
        condition=IfCondition(LaunchConfiguration('rs_enable'))
    )

    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']
    )

    rtabmap_odom = Node(
        package='rtabmap_odom',
        executable='stereo_odometry',
        output='screen',
        parameters=rtabmap_parameters,
        remappings=rtabmap_remap_topics,
    )

    rtapmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        condition=IfCondition(LaunchConfiguration('slam_enable')),
        parameters=rtabmap_parameters,
        remappings=rtabmap_remap_topics,
        arguments=['-d']
    )

    rtabmap_madwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        output='screen',
        parameters=[
            {'use_mag': LaunchConfiguration('use_mag')},
            {'world_frame': LaunchConfiguration('world_frame')}, 
            {'publish_tf': LaunchConfiguration('publish_tf_madgwick')}
        ],
        remappings=[('imu/data_raw', topic_prefix + '/imu')]
    )

    rtabmap_rviz = Node(package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=rtabmap_parameters,
        remappings=rtabmap_remap_topics,
        condition=IfCondition(LaunchConfiguration('rviz_enable'))
    )
    return LaunchDescription(declare_args + [
        rs_launch,
        rtabmap_odom,
        rtapmap_slam,
        rtabmap_madwick,
        static_tf,
        rtabmap_rviz
    ])
