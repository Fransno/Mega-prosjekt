from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='error_handler',
            executable='error_handler_node',
            name='error_handler_node',
        )
    ])