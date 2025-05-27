from setuptools import find_packages, setup

package_name = 'error_handler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/error_handler_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frans',
    maintainer_email='TeemoTeemo123@outlook.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'error_handler_node = error_handler.error_handler_node:main',
        ],
    },
)
