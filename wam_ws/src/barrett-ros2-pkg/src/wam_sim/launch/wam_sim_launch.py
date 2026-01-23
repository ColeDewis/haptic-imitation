import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    dof_arg = DeclareLaunchArgument('dof', default_value='7', description='Degrees of Freedom: 4 or 7')
    bhand_arg = DeclareLaunchArgument('bhand', default_value='false', description='Enable Barrett Hand')
    fts_arg = DeclareLaunchArgument('fts', default_value='false', description='Enable Force-Torque Sensor')

    # Launch configuration substitutions
    dof = LaunchConfiguration('dof')
    bhand = LaunchConfiguration('bhand')
    fts = LaunchConfiguration('fts')

    # Determine URDF and RViz config paths
    urdf_file = os.path.join(get_package_share_directory('wam_sim'), 'launch', 'wam7dof', 'wam7dof.urdf')
    rviz_config_file = os.path.join(get_package_share_directory('wam_sim'), 'config', 'wam_config.rviz')

    # Using 7DOF without Bhand/FTS

    return LaunchDescription([
        dof_arg,
        bhand_arg,
        fts_arg,
        # Robot State Publisher to publish TF from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        # RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])