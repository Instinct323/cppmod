from launch import LaunchDescription
from launch.actions import *
from launch_ros.actions import *


def generate_launch_description():
    display = SetEnvironmentVariable(
        name="DISPLAY",
        value="host.docker.internal:0"
    )
    node1 = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )
    return LaunchDescription([display, node1])
