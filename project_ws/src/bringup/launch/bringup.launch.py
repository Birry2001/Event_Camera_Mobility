import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include_launch(package_name: str, launch_file: str) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), "launch", launch_file)
        )
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            _include_launch("dv_ros2_capture", "capture.launch.py"),
            _include_launch("datasync_3_0", "motion_compensation.launch.py"),
            _include_launch("event_segmentation", "segmentation.launch.py"),
            _include_launch("event_clustering", "clustering.launch.py"),
        ]
    )
