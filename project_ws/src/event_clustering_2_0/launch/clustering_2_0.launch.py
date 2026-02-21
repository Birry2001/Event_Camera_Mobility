from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("event_clustering_2_0"),
        "config",
        "clustering_2_0.yaml",
    )

    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Clustering 2.0 params file",
        ),
        DeclareLaunchArgument("input_mask_topic", default_value="/event_mask"),
        DeclareLaunchArgument("events_comp_topic", default_value="/events_compensated"),
        DeclareLaunchArgument("clusters_color_topic", default_value="/event_clusters_color"),
        DeclareLaunchArgument("sync_queue_size", default_value="3"),
        DeclareLaunchArgument("sync_slop", default_value="0.05"),
        DeclareLaunchArgument("log_stats", default_value="false"),
        DeclareLaunchArgument("use_input_mask", default_value="true"),
        DeclareLaunchArgument("input_mask_min_value", default_value="1"),
        DeclareLaunchArgument("mask_fallback_enable", default_value="true"),
        DeclareLaunchArgument("mask_fallback_min_selected", default_value="15"),
        DeclareLaunchArgument("mask_fallback_on_empty_clusters", default_value="true"),
        DeclareLaunchArgument("median_filter_enable", default_value="false"),
        DeclareLaunchArgument("median_filter_kernel", default_value="1"),
        DeclareLaunchArgument("max_events", default_value="20"),
        DeclareLaunchArgument("flow_search_radius_px", default_value="3.0"),
        DeclareLaunchArgument("flow_dt_min_s", default_value="0.00001"),
        DeclareLaunchArgument("flow_dt_max_s", default_value="0.015"),
        DeclareLaunchArgument("flow_velocity_clip", default_value="5000.0"),
        DeclareLaunchArgument("velocity_scale", default_value="1.0"),
        DeclareLaunchArgument("position_time_scale", default_value="1000.0"),
        DeclareLaunchArgument("dbscan_eps", default_value="10.0"),
        DeclareLaunchArgument("dbscan_min_samples", default_value="3"),
        DeclareLaunchArgument("dbscan_wp", default_value="1.0"),
        DeclareLaunchArgument("dbscan_wv", default_value="0.002"),
        DeclareLaunchArgument("min_cluster_events", default_value="10"),
        DeclareLaunchArgument("draw_each_cluster_color", default_value="true"),
    ]

    params_override = {
        "input_mask_topic": LaunchConfiguration("input_mask_topic"),
        "events_comp_topic": LaunchConfiguration("events_comp_topic"),
        "clusters_color_topic": LaunchConfiguration("clusters_color_topic"),
        "sync_queue_size": ParameterValue(LaunchConfiguration("sync_queue_size"), value_type=int),
        "sync_slop": ParameterValue(LaunchConfiguration("sync_slop"), value_type=float),
        "log_stats": ParameterValue(LaunchConfiguration("log_stats"), value_type=bool),
        "use_input_mask": ParameterValue(LaunchConfiguration("use_input_mask"), value_type=bool),
        "input_mask_min_value": ParameterValue(
            LaunchConfiguration("input_mask_min_value"), value_type=int
        ),
        "mask_fallback_enable": ParameterValue(
            LaunchConfiguration("mask_fallback_enable"), value_type=bool
        ),
        "mask_fallback_min_selected": ParameterValue(
            LaunchConfiguration("mask_fallback_min_selected"), value_type=int
        ),
        "mask_fallback_on_empty_clusters": ParameterValue(
            LaunchConfiguration("mask_fallback_on_empty_clusters"), value_type=bool
        ),
        "median_filter_enable": ParameterValue(
            LaunchConfiguration("median_filter_enable"), value_type=bool
        ),
        "median_filter_kernel": ParameterValue(
            LaunchConfiguration("median_filter_kernel"), value_type=int
        ),
        "max_events": ParameterValue(LaunchConfiguration("max_events"), value_type=int),
        "flow_search_radius_px": ParameterValue(
            LaunchConfiguration("flow_search_radius_px"), value_type=float
        ),
        "flow_dt_min_s": ParameterValue(LaunchConfiguration("flow_dt_min_s"), value_type=float),
        "flow_dt_max_s": ParameterValue(LaunchConfiguration("flow_dt_max_s"), value_type=float),
        "flow_velocity_clip": ParameterValue(
            LaunchConfiguration("flow_velocity_clip"), value_type=float
        ),
        "velocity_scale": ParameterValue(LaunchConfiguration("velocity_scale"), value_type=float),
        "position_time_scale": ParameterValue(
            LaunchConfiguration("position_time_scale"), value_type=float
        ),
        "dbscan_eps": ParameterValue(LaunchConfiguration("dbscan_eps"), value_type=float),
        "dbscan_min_samples": ParameterValue(
            LaunchConfiguration("dbscan_min_samples"), value_type=int
        ),
        "dbscan_wp": ParameterValue(LaunchConfiguration("dbscan_wp"), value_type=float),
        "dbscan_wv": ParameterValue(LaunchConfiguration("dbscan_wv"), value_type=float),
        "min_cluster_events": ParameterValue(
            LaunchConfiguration("min_cluster_events"), value_type=int
        ),
        "draw_each_cluster_color": ParameterValue(
            LaunchConfiguration("draw_each_cluster_color"), value_type=bool
        ),
    }

    return LaunchDescription(
        args
        + [
            Node(
                package="event_clustering_2_0",
                executable="event_clustering_2_0_node",
                name="event_clustering_2_0",
                output="screen",
                parameters=[LaunchConfiguration("params_file"), params_override],
            )
        ]
    )
