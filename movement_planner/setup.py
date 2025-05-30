from setuptools import find_packages, setup

package_name = 'movement_planner'

setup(
    name=package_name,
    version='0.0.0',
    
    # Automatisk finn alle Python-pakker unntatt 'test'
    packages=find_packages(exclude=['test']),

    # Data som skal inkluderes i installasjonen
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),  # For ament index
        ('share/' + package_name, ['package.xml']),  # ROS2 metadata
        ('share/' + package_name + '/launch', ['launch/movement_planner_launch.py']),  # Launch-fil
    ],

    # Avhengigheter for installasjon
    install_requires=['setuptools'],
    zip_safe=True,

    # Vedlikeholderinfo
    maintainer='frans',
    maintainer_email='TeemoTeemo123@outlook.com',

    # Kort beskrivelse og lisens
    description='TODO: Package description',
    license='Apache-2.0',

    # Testkrav
    tests_require=['pytest'],

    # Hvilke konsollkommandoer som skal være tilgjengelige etter installasjon
    entry_points={
        'console_scripts': [
            'movement_planner_node = movement_planner.movement_planner_node:main',
        ],
    },
)
