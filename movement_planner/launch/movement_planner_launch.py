from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='movement_planner',
            executable='movement_planner_node',
            name='movement_planner_node',
        )
    ])