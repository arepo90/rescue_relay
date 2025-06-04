from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rescue_relay',
            executable='relay',
            name='relay_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        )
    ])
