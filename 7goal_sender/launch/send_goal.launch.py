from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    x = LaunchConfiguration('x', default='1.0')
    y = LaunchConfiguration('y', default='0.5')
    yaw_deg = LaunchConfiguration('yaw_deg', default='0.0')
    frame_id = LaunchConfiguration('frame_id', default='map')

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='1.0'),
        DeclareLaunchArgument('y', default_value='0.5'),
        DeclareLaunchArgument('yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        Node(
            package='nav2_goal_sender',
            executable='nav2_goal_client',
            name='nav2_goal_client',
            parameters=[{
                'x': x,
                'y': y,
                'yaw_deg': yaw_deg,
                'frame_id': frame_id,
            }],
            output='screen'
        )
    ])
