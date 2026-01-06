from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='/dvs/events',
            description='dvs_msgs/EventArray input topic'
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/events',
            description='dv_ros2_msgs/EventArray output topic'
        ),
        Node(
            package='dvs_to_dv_bridge',
            executable='dvs_to_dv_bridge_node',
            name='dvs_to_dv_bridge',
            parameters=[{
                'input_topic': input_topic,
                'output_topic': output_topic,
            }],
            output='screen',
            emulate_tty=True,
        ),
    ])
