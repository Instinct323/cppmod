from launch import LaunchDescription
from launch.actions import *
from launch_ros.actions import *

ENVIRON = {"DISPLAY": "host.docker.internal:0"}


def generate_launch_description():
    SetLaunchConfiguration
    return LaunchDescription([
        Node(
            package="turtle_sim",
            executable="turtlesim_node",
            parameters=[ENVIRON]
        )
    ])
