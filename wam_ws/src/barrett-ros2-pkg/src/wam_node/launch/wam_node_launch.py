from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="wam_node",
            executable="wam_node",
            name="wam_node",
            output="screen",
            emulate_tty=True
        )
    ])
