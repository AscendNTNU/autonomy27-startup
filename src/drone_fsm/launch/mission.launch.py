from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_fsm',
            executable='controller_sim_node',
            name='controller_sim_node',
            output='screen',
        ),

        Node(
            package='drone_fsm',
            executable='takeoff_node',
            name='takeoff_node',
            output='screen',
        ),

        Node(
            package='drone_fsm',
            executable='waypoint_node',
            name='waypoint_node',
            output='screen',
        ),

        Node(
            package='drone_fsm',
            executable='circle',
            name='circle',
            output='screen',
        ),

        Node(
            package='drone_fsm',
            executable='go_to_node',
            name='go_to_node',
            output='screen',
        ),

        Node(
            package='drone_fsm',
            executable='landing_node',
            name='landing_node',
            output='screen',
        ),
    ])