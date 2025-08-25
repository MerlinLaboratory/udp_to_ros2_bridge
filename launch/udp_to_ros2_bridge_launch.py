from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='udp_to_ros2_bridge',               # Replace with your package name
            executable='udp_to_ros2_bridge',       # Node executable
            name='udp_to_ros2_bridge',                     # ROS 2 node name
            parameters=['config/comm_param.yaml'],      # YAML file path
            output='screen'                             # Print logs to terminal
        )
    ])