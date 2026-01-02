from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory("articubot_one")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "launch_sim.launch.py")
        )
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "online_async_launch.py")
        )
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "navigation_launch.py")
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "autonomy.launch.py")
        )
    )

    return LaunchDescription([
        sim,
        TimerAction(period=6.0, actions=[slam]),
        TimerAction(period=10.0, actions=[navigation]),
        TimerAction(period=15.0, actions=[autonomy])
    ])
