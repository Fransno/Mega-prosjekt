from setuptools import setup
import os
from glob import glob

# Navnet på ROS2-pakken
package_name = 'ur_mover'

setup(
    name=package_name,
    version='0.0.0',  # Versjon av pakken, endres ved behov
    packages=[package_name],  # Inkluderer Python-modulen med samme navn som pakken
    data_files=[
        # Installasjon av package.xml til riktig ROS2-katalog
        ('share/' + package_name, ['package.xml']),
        
        # Inkluderer launch-filer fra src/launch/ inn i riktig katalogstruktur
        (os.path.join('share', package_name, 'launch'), glob('src/launch/*.launch.py')),
    ],
    install_requires=['setuptools'],  # Avhenger kun av setuptools for installasjon
    zip_safe=True,  # Tillater installasjon som zip (ingen krav til å pakke ut)
    maintainer='Jørgen N. Melling',  
    maintainer_email='din@email.com', 
    description='Node som sender posisjoner til UR-robot og gir status',  # Kort beskrivelse
    license='MIT',  # Lisens for koden
    tests_require=['pytest'],  # Test-avhengigheter
    entry_points={
        'console_scripts': [
            # Registrerer kjørbar node: gir 'ros2 run ur_mover ur_mover_node'
            'ur_mover_node = ur_mover.ur_mover_node:main',
        ],
    },
)

