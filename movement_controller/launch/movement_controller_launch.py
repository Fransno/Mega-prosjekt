from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='movement_controller',
            executable='movement_controller_node',
            name='movement_controller_node',
        )
    ])