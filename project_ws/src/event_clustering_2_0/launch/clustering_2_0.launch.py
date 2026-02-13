from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("event_clustering_2_0"),
        "config",
        "clustering_2_0.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Clustering 2.0 params file",
            ),
            Node(
                package="event_clustering_2_0",
                executable="event_clustering_2_0_node",
                name="event_clustering_2_0",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
