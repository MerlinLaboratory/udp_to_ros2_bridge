import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_dir = get_package_share_directory('udp_to_ros2_bridge')

    config_path_cmd = DeclareLaunchArgument(
        'config_path',
        default_value=pkg_dir + '/config/comm_param.yaml',
        description='Full path to the config file')

    node = Node(
            package='udp_to_ros2_bridge',               # Replace with your package name
            executable='udp_to_ros2_bridge',       # Node executable
            name='udp_to_ros2_bridge',                     # ROS 2 node name
            parameters=[
                LaunchConfiguration('config_path'),
        ])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(config_path_cmd)
    ld.add_action(node)
    return ld