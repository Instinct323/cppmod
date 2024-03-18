from launch import LaunchDescription
from launch.actions import *
from launch_ros.actions import *


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            name="DISPLAY",
            value="host.docker.internal:0"
        ),
        Node(
            package="turtlesim",
            executable="turtlesim_node"
        ),
    ])
