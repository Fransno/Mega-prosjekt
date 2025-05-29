import os
import json
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Last inn konfigurasjon fra JSON-fil
with open('config.json') as file:
    data = json.load(file)

# Hent nest høyeste video-enhet (for eksempel /dev/video1)
def get_second_highest_video_device():
    video_devices = [int(dev.replace('video', '')) for dev in os.listdir('/dev') if dev.startswith('video')]
    video_devices.sort(reverse=True)
    if len(video_devices) > 1:
        return f"/dev/video{video_devices[1]}"
    elif video_devices:
        return f"/dev/video{video_devices[0]}"
    else:
        raise RuntimeError("No video devices found!")

# Generer launch-beskrivelse
def generate_launch_description():

    # Hent riktig videodevice
    video_device = get_second_highest_video_device()

    # Hent kamerainnstillinger fra config
    autoexposure = data["pipeline.launch.py"]["autoexposure"]
    exposure_absolute = data["pipeline.launch.py"]["exposure_absolute"]
    auto_white_balance = data["pipeline.launch.py"]["auto_white_balance"]
    white_balance_temperature = data["pipeline.launch.py"]["white_balance_temperature"]
    brightness = data["pipeline.launch.py"]["brightness"]
    contrast = data["pipeline.launch.py"]["contrast"]
    saturation = data["pipeline.launch.py"]["saturation"]
    gain = data["pipeline.launch.py"]["gain"]

    # Lag liste over kameraparamentere
    camera_params = [
        {'autoexposure': autoexposure},
        {'exposure_absolute': exposure_absolute},
        {'auto_white_balance': auto_white_balance},
        {'white_balance_temperature': white_balance_temperature}, 
        {'brightness': brightness},
        {'contrast': contrast},
        {'saturation': saturation},
        {'gain': gain},
    ]

    return LaunchDescription([
        # Start USB-kamera med parametere og kamera-info
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[
                {'video_device': video_device},
                {'camera_info_url': 'file://' + os.path.join(get_package_share_directory('camera_pipeline'), 'config', 'default_cam.yaml')}   
            ] + camera_params
        ),

        # Start bildebehandlingsnoden (color_detector)
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
