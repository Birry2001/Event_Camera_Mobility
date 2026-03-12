from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory("event_nav2_layer"),
        "config",
        "event_nav2_layer.yaml",
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Costmap params file including event_nav2_layer plugin",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically activate lifecycle nodes",
            ),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/dynamic_obstacles",
                description="PointCloud2 topic from event_clustering_2_0",
            ),
            Node(
                package="nav2_costmap_2d",
                executable="nav2_costmap_2d",
                name="event_costmap",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "event_layer.pointcloud_topic": pointcloud_topic,
                    },
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_event_costmap",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "autostart": ParameterValue(autostart, value_type=bool),
                        "node_names": ["event_costmap"],
                    }
                ],
            ),
        ]
    )
