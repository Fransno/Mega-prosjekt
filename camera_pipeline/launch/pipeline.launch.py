from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def get_second_highest_video_device():
    # Check available video devices in /dev
    video_devices = [int(dev.replace('video', '')) for dev in os.listdir('/dev') if dev.startswith('video')]
    video_devices.sort(reverse=True)  # Sort in descending order
    if len(video_devices) > 1:
        return f"/dev/video{video_devices[1]}"  # Second-highest device
    elif video_devices:
        return f"/dev/video{video_devices[0]}"  # Fallback to the highest if only one exists
    else:
        raise RuntimeError("No video devices found!")

def generate_launch_description():
    # Get the second-highest video device
    video_device = get_second_highest_video_device()

    # Path to the parameter file
    param_file_path = os.path.join(
        get_package_share_directory('camera_pipeline'),
        'config',
        'color_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{'video_device': video_device}]
        ),
        Node(
            package='camera_pipeline',
            executable='color_detector',
            name='color_detector',
            output='screen',
            parameters=[param_file_path],
            remappings=[
                ('image_raw', '/image_raw'),
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'install/camera_pipeline/share/camera_pipeline/rviz/cube_task_controller.rviz'],
            output='screen'
        )
    ])