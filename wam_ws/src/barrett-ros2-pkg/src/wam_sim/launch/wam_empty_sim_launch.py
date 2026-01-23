import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Launch arguments
    dof_arg = DeclareLaunchArgument('dof', default_value='7', description='Degrees of freedom (4 or 7)')
    bhand_arg = DeclareLaunchArgument('bhand', default_value='false', description='Enable Barrett Hand')
    fts_arg = DeclareLaunchArgument('fts', default_value='false', description='Enable Force/Torque Sensor')

    def launch_nodes(context, *args, **kwargs):
        dof = LaunchConfiguration('dof').perform(context)
        bhand = LaunchConfiguration('bhand').perform(context) == 'true'
        fts = LaunchConfiguration('fts').perform(context) == 'true'

        urdf = ''
        rviz_config = ''

        # Determine URDF and RViz config based on inputs
        if dof == "4" and not fts and not bhand:
            urdf = os.path.join(get_package_share_directory('wam_sim'),'launch','wam4dof','wam4dof.urdf')
            rviz_config = os.path.join(get_package_share_directory('wam_sim'),'config','wam_config.rviz')
        elif dof == "7" and not fts and not bhand:
            urdf = os.path.join(get_package_share_directory('wam_sim'),'launch','wam7dof','wam7dof.urdf')
            rviz_config = os.path.join(get_package_share_directory('wam_sim'),'config','wam_config.rviz')
        elif dof == "4" and fts and not bhand:
            urdf = os.path.join(get_package_share_directory('wam_sim'),'launch','wam4dof','wam4dof_fts.urdf')
            rviz_config = os.path.join(get_package_share_directory('wam_sim'),'config','wam_config.rviz')
        elif dof == "7" and fts and not bhand:
            urdf = os.path.join(get_package_share_directory('wam_sim'),'launch','wam7dof','wam7dof_fts.urdf')
            rviz_config = os.path.join(get_package_share_directory('wam_sim'),'config','wam_config.rviz')
        elif dof == "4" and not fts and bhand:
            urdf = os.path.join(get_package_share_directory('wam_sim'),'launch','wam4dof','wam4dof_hand.urdf')
            rviz_config = os.path.join(get_package_share_directory('wam_sim'),'config','wam_config.rviz')
        elif dof == "7" and not fts and bhand:
            urdf = os.path.join(get_package_share_directory('wam_sim'),'launch','wam7dof','wam7dof_hand.urdf')
            rviz_config = os.path.join(get_package_share_directory('wam_sim'),'config','wam_config.rviz')
        elif dof == "4" and fts and bhand:
            urdf = os.path.join(get_package_share_directory('wam_sim'),'launch','wam4dof','wam4dof_hand_fts.urdf')
            rviz_config = os.path.join(get_package_share_directory('wam_sim'),'config','wam_config.rviz')
        elif dof == "7" and fts and bhand:
            urdf = os.path.join(get_package_share_directory('wam_sim'),'launch','wam7dof','wam7dof_hand_fts.urdf')
            rviz_config = os.path.join(get_package_share_directory('wam_sim'),'config','wam_config.rviz')
        elif dof == "0" and bhand:
            urdf = os.path.join(get_package_share_directory('wam_sim'),'launch','wam7dof','barrett_hand.urdf')
            rviz_config = os.path.join(get_package_share_directory('wam_sim'),'config','bhand_config.rviz')
        else:
            print("Invalid configuration specified.\nUsage: ros2 launch wam_sim wam_empty_sim.launch.py dof:=<4 or 7> fts:=<true/false> bhand:=<true/false>")
            return []

        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                arguments=[urdf]
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config]
            )
        ]

    return LaunchDescription([
        dof_arg,
        bhand_arg,
        fts_arg,
        OpaqueFunction(function=launch_nodes)
    ])