from setuptools import find_packages, setup
import os # Lagt til for launch filer og config filer
from glob import glob # Lagt til for launch filer og config filer

package_name = 'camera_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    # Bruk find_packages() for å automatisk finne pakken(e)
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Installer package.xml
        ('share/' + package_name, ['package.xml']),
        # Installer launch-filer (bruk os.path.join og glob for fleksibilitet)
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Installer config-filer (hvis du har en config-mappe)
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # Installer RViz-filer (hvis du har en rviz-mappe)
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oscar', # Bytt ut med ditt navn/e-post hvis ønskelig
    maintainer_email='oscar@todo.todo', # Bytt ut med ditt navn/e-post hvis ønskelig
    description='TODO: Package description', # Gi en bedre beskrivelse
    license='TODO: License declaration', # Velg en lisens, f.eks. Apache License 2.0
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Eksisterende noder fra ditt eksempel
            'gaussian_blur = camera_pipeline.gaussian_blur:main',
            'canny_edge = camera_pipeline.canny_edge:main',
            # Din fargedetektor
            'color_detector = camera_pipeline.color_detector:main',
            # Den nye styringsnoden
            'cube_task_controller = camera_pipeline.cube_task_controller:main',
        ],
    },
)