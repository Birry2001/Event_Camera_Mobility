from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    package_name = 'datasync_3_0'
    config_path = os.path.join(get_package_share_directory(package_name), 'config', 'motion_compensation.yaml')

    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)

    events_topic = LaunchConfiguration('events_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    count_image_topic = LaunchConfiguration('count_image_topic')
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
            'count_image_topic',
            default_value='count_image',
            description='Output count image topic name'
        ),
        Node(
            package=package_name,
            executable='motion_compensation_node',
            name='datasync_node',
            parameters=[params_file, {
                'events_topic': events_topic,
                'imu_topic': imu_topic,
                'count_image_topic': count_image_topic,
            }],
            output='screen',
            emulate_tty=True,
        ),
    ])
