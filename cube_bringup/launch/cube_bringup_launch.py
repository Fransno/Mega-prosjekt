import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# Launch-fil for å starte hele systemet samlet
def generate_launch_description():
    return LaunchDescription([
        # Inkluder kamera-pipeline (kamera + fargedeteksjon)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('camera_pipeline'), 'launch', 'pipeline.launch.py')
            )
        ),
        # Inkluder feilhåndteringsnoden
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('error_handler'), 'launch', 'error_handler_launch.py')
            )
        ),
        # Inkluder planleggingsnoden (genererer posisjonssekvens)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('movement_planner'), 'launch', 'movement_planner_launch.py')
            )
        ),
        # Inkluder kontrollnoden (sender bevegelseskommandoer til robot)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('movement_controller'), 'launch', 'movement_controller_launch.py')
            )
        ),
    ])
