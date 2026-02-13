from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("event_clustering"),
        "config",
        "clustering.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Object contours extraction params file",
            ),
            Node(
                package="event_clustering",
                executable="event_clustering_node",
                name="object_contours_from_mask",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
