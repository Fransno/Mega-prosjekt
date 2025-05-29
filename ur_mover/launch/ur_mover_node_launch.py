from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur_mover',  
            executable='ur_mover_node',  
            name='ur_mover_node',
            output='screen'
        )
    ])

