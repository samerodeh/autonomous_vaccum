import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_name = "articubot_one"
    pkg_share = get_package_share_directory(package_name)

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'slam_test.sdf'),
        description='Path to world file'
    )
    world = LaunchConfiguration('world')

    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(pkg_share, "description", "robot.urdf.xacro"),
            " use_ros2_control:=true sim_mode:=true"
        ]),
        value_type=str
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }],
        output="screen"
    )

    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-s", "--headless-rendering", world],
        output="screen"
    )

    spawn_robot = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=[
            "-name", "articubot",
            "-topic", "robot_description",
            "-x", "0",
            "-y", "0",
            "-z", "0.1"
        ],
        output="screen"
    )

    bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')],
        output="screen"
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        rsp,
        jsp,
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=5.0, actions=[bridge]),
        TimerAction(period=6.0, actions=[ekf]),
    ])
