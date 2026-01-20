from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = (
        get_package_share_directory("event_rate_filter") + "/config/rate_filter.yaml"
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to rate filter params YAML",
            ),
            Node(
                package="event_rate_filter",
                executable="event_rate_filter_node",
                name="event_rate_filter_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
