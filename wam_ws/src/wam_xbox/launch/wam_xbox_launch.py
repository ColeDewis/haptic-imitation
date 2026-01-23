from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # TODO: add config files for control as arguments

    # Define the node
    wam_xbox_control = Node(
        package="wam_xbox",  # Ensure this matches your package name in setup.py
        executable="xbox_node",
        name="xbox_node",
        output="screen",
    )

    joy = Node(
        package="joy",  # Ensure this matches your package name in setup.py
        executable="joy_node",
        name="joy_node",
    )

    return LaunchDescription(
        [
            wam_xbox_control,
            joy,
        ]
    )
