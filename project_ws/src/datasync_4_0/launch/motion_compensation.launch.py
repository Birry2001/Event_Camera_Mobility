from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    package_name = 'datasync_4_0'
    config_path = os.path.join(get_package_share_directory(package_name), 'config', 'motion_compensation.yaml')

    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)

    events_topic = LaunchConfiguration('events_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    count_image_topic = LaunchConfiguration('count_image_topic')
    base_frame = LaunchConfiguration('base_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=config_path,
            description='Path to a YAML file with node parameters'
        ),
        DeclareLaunchArgument(
            'events_topic',
            default_value='events',
            description='EventArray topic name'
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='imu',
            description='IMU topic name'
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='depth',
            description='Depth image topic name'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='camera_info',
            description='CameraInfo topic name'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='odom',
            description='Odometry topic name'
        ),
        DeclareLaunchArgument(
            'count_image_topic',
            default_value='count_image',
            description='Output count image topic name'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Base frame id'
        ),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera',
            description='Event camera frame id'
        ),
        Node(
            package=package_name,
            executable='motion_compensation_node',
            name='datasync_4_0_node',
            parameters=[params_file, {
                'events_topic': events_topic,
                'imu_topic': imu_topic,
                'depth_topic': depth_topic,
                'camera_info_topic': camera_info_topic,
                'odom_topic': odom_topic,
                'count_image_topic': count_image_topic,
                'base_frame': base_frame,
                'camera_frame': camera_frame,
            }],
            output='screen',
            emulate_tty=True,
        ),
    ])
