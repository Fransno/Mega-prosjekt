from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    # Finn alle Python-pakker i prosjektet (uten test-mappen)
    packages=find_packages(exclude=['test']),
    data_files=[
        # Installer metadata slik at ROS2 finner pakken
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Inkluder package.xml
        ('share/' + package_name, ['package.xml']),
        # Inkluder launch-filer (f.eks. .py, .xml eller .yaml)
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Inkluder konfigurasjonsfiler (f.eks. YAML med parametre)
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oscar',  # Oppdater med faktisk navn
    maintainer_email='oscar@todo.todo',  # Oppdater med faktisk e-post
    description='TODO: Package description',  # Kort beskrivelse av pakken
    license='TODO: License declaration',  # F.eks. 'MIT', 'Apache 2.0' osv.
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Navn på executable = modulsti:main-funksjon
            'gaussian_blur = camera_pipeline.gaussian_blur:main',
            'canny_edge = camera_pipeline.canny_edge:main',
            'color_detector = camera_pipeline.color_detector:main',
        ],
    },
)
