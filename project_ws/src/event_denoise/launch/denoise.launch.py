from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    package_name = "event_denoise"
    config_path = os.path.join(get_package_share_directory(package_name), "config", "denoise.yaml")

    with open(config_path, "r") as file:
        config = yaml.safe_load(file)

    return LaunchDescription([
        Node(
            package=package_name,
            executable="event_denoise_node",
            name="event_denoise",
            parameters=[config],
            output="screen",
            emulate_tty=True,
        ),
    ])
