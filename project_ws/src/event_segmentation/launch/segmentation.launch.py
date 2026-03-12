from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("event_segmentation"),
        "config",
        "segmentation.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Segmentation params file",
            ),
            Node(
                package="event_segmentation",
                executable="event_segmentation_node",
                name="event_segmentation",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
