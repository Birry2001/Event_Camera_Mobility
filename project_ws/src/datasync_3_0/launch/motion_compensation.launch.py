from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = 'datasync_3_0'
    config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'motion_compensation.yaml'
    )

    events_topic = LaunchConfiguration('events_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    count_image_topic = LaunchConfiguration('count_image_topic')
    lambda_a = LaunchConfiguration('lambda_a')
    lambda_b = LaunchConfiguration('lambda_b')
    prefilter_enable = LaunchConfiguration('prefilter_enable')
    prefilter_refractory_enable = LaunchConfiguration('prefilter_refractory_enable')
    prefilter_refractory_us = LaunchConfiguration('prefilter_refractory_us')
    prefilter_ba_enable = LaunchConfiguration('prefilter_ba_enable')
    prefilter_ba_radius_px = LaunchConfiguration('prefilter_ba_radius_px')
    prefilter_ba_window_us = LaunchConfiguration('prefilter_ba_window_us')
    prefilter_ba_min_neighbors = LaunchConfiguration('prefilter_ba_min_neighbors')
    prefilter_ba_same_polarity_only = LaunchConfiguration('prefilter_ba_same_polarity_only')
    prefilter_ba_support_from_kept_only = LaunchConfiguration('prefilter_ba_support_from_kept_only')
    prefilter_hot_pixel_enable = LaunchConfiguration('prefilter_hot_pixel_enable')
    prefilter_hot_pixel_max_events_per_batch = LaunchConfiguration('prefilter_hot_pixel_max_events_per_batch')
    prefilter_log_stats = LaunchConfiguration('prefilter_log_stats')
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
        DeclareLaunchArgument(
            'lambda_a',
            default_value='0.3',
            description='Lambda dynamic threshold coefficient a'
        ),
        DeclareLaunchArgument(
            'lambda_b',
            default_value='0.2',
            description='Lambda dynamic threshold coefficient b'
        ),
        DeclareLaunchArgument(
            'prefilter_enable',
            default_value='true',
            description='Enable prefilter before compensation'
        ),
        DeclareLaunchArgument(
            'prefilter_refractory_enable',
            default_value='true',
            description='Enable refractory prefilter'
        ),
        DeclareLaunchArgument(
            'prefilter_refractory_us',
            default_value='600',
            description='Refractory period in microseconds'
        ),
        DeclareLaunchArgument(
            'prefilter_ba_enable',
            default_value='true',
            description='Enable BA spatial-temporal prefilter'
        ),
        DeclareLaunchArgument(
            'prefilter_ba_radius_px',
            default_value='2',
            description='BA neighborhood radius in pixels'
        ),
        DeclareLaunchArgument(
            'prefilter_ba_window_us',
            default_value='4000',
            description='BA temporal window in microseconds'
        ),
        DeclareLaunchArgument(
            'prefilter_ba_min_neighbors',
            default_value='1',
            description='Minimum BA neighbors required'
        ),
        DeclareLaunchArgument(
            'prefilter_ba_same_polarity_only',
            default_value='false',
            description='Use only same polarity for BA support'
        ),
        DeclareLaunchArgument(
            'prefilter_ba_support_from_kept_only',
            default_value='false',
            description='If true, only kept events provide BA support'
        ),
        DeclareLaunchArgument(
            'prefilter_hot_pixel_enable',
            default_value='true',
            description='Enable hot-pixel prefilter'
        ),
        DeclareLaunchArgument(
            'prefilter_hot_pixel_max_events_per_batch',
            default_value='15',
            description='Drop pixel if batch count reaches this threshold'
        ),
        DeclareLaunchArgument(
            'prefilter_log_stats',
            default_value='false',
            description='Print prefilter statistics'
        ),
        Node(
            package=package_name,
            executable='motion_compensation_node',
            name='datasync_node',
            parameters=[params_file, {
                'events_topic': events_topic,
                'imu_topic': imu_topic,
                'count_image_topic': count_image_topic,
                'lambda_a': ParameterValue(lambda_a, value_type=float),
                'lambda_b': ParameterValue(lambda_b, value_type=float),
                'prefilter_enable': ParameterValue(prefilter_enable, value_type=bool),
                'prefilter_refractory_enable': ParameterValue(prefilter_refractory_enable, value_type=bool),
                'prefilter_refractory_us': ParameterValue(prefilter_refractory_us, value_type=int),
                'prefilter_ba_enable': ParameterValue(prefilter_ba_enable, value_type=bool),
                'prefilter_ba_radius_px': ParameterValue(prefilter_ba_radius_px, value_type=int),
                'prefilter_ba_window_us': ParameterValue(prefilter_ba_window_us, value_type=int),
                'prefilter_ba_min_neighbors': ParameterValue(prefilter_ba_min_neighbors, value_type=int),
                'prefilter_ba_same_polarity_only': ParameterValue(prefilter_ba_same_polarity_only, value_type=bool),
                'prefilter_ba_support_from_kept_only': ParameterValue(prefilter_ba_support_from_kept_only, value_type=bool),
                'prefilter_hot_pixel_enable': ParameterValue(prefilter_hot_pixel_enable, value_type=bool),
                'prefilter_hot_pixel_max_events_per_batch': ParameterValue(prefilter_hot_pixel_max_events_per_batch, value_type=int),
                'prefilter_log_stats': ParameterValue(prefilter_log_stats, value_type=bool),
            }],
            output='screen',
            emulate_tty=True,
        ),
    ])
