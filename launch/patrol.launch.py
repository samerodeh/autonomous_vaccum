from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='articubot_one',
            executable='patrol_node.py',
            name='patrol_node',
            output='screen'
        ),
    ])
