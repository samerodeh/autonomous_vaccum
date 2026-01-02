from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    frontier_explorer = Node(
        package='articubot_one',
        executable='frontier_explorer.py',
        name='frontier_explorer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        frontier_explorer
    ])

