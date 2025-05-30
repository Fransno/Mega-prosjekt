from setuptools import find_packages, setup

package_name = 'cube_bringup'

setup(
    name=package_name,
    version='0.0.0',
    # Finn alle Python-pakker i mappen (unntatt 'test')
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS2 krever at pakken registreres i ament_index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Legg til launch-filen i riktig katalog
        ('share/' + package_name + '/launch', ['launch/cube_bringup_launch.py']),
        # Inkluder package.xml for ROS2 metadata
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frans', 
    maintainer_email='TeemoTeemo123@outlook.com',
    description='Launch all cube nodes',  # Kort og tydelig beskrivelse
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Ingen noder kjøres direkte fra denne pakken, så denne er tom
        ],
    },
)

