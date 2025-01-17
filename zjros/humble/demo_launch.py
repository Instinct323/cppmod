from launch import LaunchDescription
from launch.actions import *
from launch_ros.actions import *


def generate_launch_description():
    node1 = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )
    return LaunchDescription([display, node1])
