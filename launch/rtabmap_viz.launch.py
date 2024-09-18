from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
        'frame_id':'camera_link',
        'subscribe_stereo':True,
        'subscribe_odom_info':True,
        'wait_imu_to_init':True}]

    remappings=[
        ('imu', '/imu/data'),
        ('left/image_rect', '/camera/infra1/image_rect_raw'),
        ('left/camera_info', '/camera/infra1/camera_info'),
        ('right/image_rect', '/camera/infra2/image_rect_raw'),
        ('right/camera_info', '/camera/infra2/camera_info')]

    return LaunchDescription([

        Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=parameters,
        remappings=remappings),
    ])
