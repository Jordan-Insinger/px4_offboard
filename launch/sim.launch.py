import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_params = os.path.join(get_package_share_directory('px4_offboard'), 'param', 'joy_config.yaml')

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace='px4_sim',
            parameters=[joy_params],
            output='screen'
        ),
        Node(
            package='px4_offboard',
            executable='px4_offboard_node',
            name='px4_offboard_node',
            namespace='px4_sim',
            parameters=[
                os.path.join(get_package_share_directory('px4_offboard'), 'param', 'xbox_controller.yaml'),
                os.path.join(get_package_share_directory('astro_telemetry'), 'param', 'park_coordinates.yaml')],
            output='screen'
        )
    ])
