from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def get_second_highest_video_device():
    video_devices = [int(dev.replace('video', '')) for dev in os.listdir('/dev') if dev.startswith('video')]
    video_devices.sort(reverse=True)
    if len(video_devices) > 1:
        return f"/dev/video{video_devices[1]}"
    elif video_devices:
        return f"/dev/video{video_devices[0]}"
    else:
        raise RuntimeError("No video devices found!")

def generate_launch_description():

    video_device = get_second_highest_video_device()

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[
                {'video_device': video_device},
                {'camera_info_url': 'file://' + os.path.join(get_package_share_directory('camera_pipeline'), 'config', 'default_cam.yaml')}   
            ]        
        ),

        Node(
            package='camera_pipeline',
            executable='color_detector',
            name='color_detector',
            output='screen',
            remappings=[
                ('image_raw', '/image_raw'),
            ]
        )
    ])