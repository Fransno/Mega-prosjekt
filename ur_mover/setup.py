from setuptools import setup
import os
from glob import glob

package_name = 'ur_mover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # Inkluder launch-filer fra src/launch/
        (os.path.join('share', package_name, 'launch'), glob('src/launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jørgen N. Melling',
    maintainer_email='din@email.com',
    description='Node som sender posisjoner til UR-robot og gir status',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur_mover_node = ur_mover.ur_mover_node:main',
        ],
    },
)
