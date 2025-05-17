from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package = 'joy',
            executable = 'joy_node'
        ),
        Node(
            name = 'mission_control',
            package = 'gui_interface',
            executable = 'main',
            output = 'screen'
        )
    ])
