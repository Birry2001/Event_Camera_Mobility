from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = (
        get_package_share_directory("event_hot_pixel_mask")
        + "/config/hot_pixel_mask.yaml"
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to hot pixel mask params YAML",
            ),
            Node(
                package="event_hot_pixel_mask",
                executable="hot_pixel_mask_node",
                name="hot_pixel_mask_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
