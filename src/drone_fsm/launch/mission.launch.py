"""Start the whole mission (except the simulator) with one command.

Start the simulator first in its own terminal:
    cd ~/PX4-Autopilot && HEADLESS=1 make px4_sitl gz_x500

Then:
    ros2 launch drone_fsm mission.launch.py                     # square (default)
    ros2 launch drone_fsm mission.launch.py figure:=circle
    ros2 launch drone_fsm mission.launch.py start_agent:=false  # agent already running
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# TODO: check with `ros2 pkg executables <package>` and change if needed
PACKAGE = 'drone_fsm'
GO_TO_EXECUTABLE = 'go_to_node'
TAKEOFF_EXECUTABLE = 'takeoff_node'
LANDING_EXECUTABLE = 'landing_node'

# Seconds to wait before takeoff starts, so the agent has connected to PX4
# and the other nodes are ready to receive messages
TAKEOFF_DELAY = 5.0


def generate_launch_description():
    figure = LaunchConfiguration('figure')
    start_agent = LaunchConfiguration('start_agent')

    return LaunchDescription([
        DeclareLaunchArgument(
            'figure', default_value='square',
            description='Which figure to fly (name of the executable in setup.py)'),
        DeclareLaunchArgument(
            'start_agent', default_value='true',
            description='Start MicroXRCEAgent (set to false if it is already running)'),

        # Bridge between PX4 and ROS 2
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen',
            condition=IfCondition(start_agent)),

        # Nodes that wait for something: start these first
        Node(package=PACKAGE, executable=LANDING_EXECUTABLE, output='screen'),
        Node(package=PACKAGE, executable=GO_TO_EXECUTABLE, output='screen'),
        Node(package=PACKAGE, executable=figure, output='screen'),

        # Takeoff starts everything, so it goes last
        TimerAction(
            period=TAKEOFF_DELAY,
            actions=[Node(package=PACKAGE, executable=TAKEOFF_EXECUTABLE, output='screen')]),
    ])