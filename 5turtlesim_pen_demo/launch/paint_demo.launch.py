from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node'
        ),
        Node(
            package='turtlesim_pen_demo',
            executable='move_and_paint',
            name='turtlesim_pen_demo',
            parameters=[{
                'r': 0, 'g': 255, 'b': 0,   # green
                'width': 2,
                'off': 0
            }]
        ),
    ])
