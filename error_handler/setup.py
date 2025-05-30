from setuptools import find_packages, setup

package_name = 'error_handler'  # Navnet på ROS2-pakken

setup(
    name=package_name,
    version='0.0.0',
    
    # Inkluderer alle Python-moduler i pakken, unntatt test/
    packages=find_packages(exclude=['test']),
    
    # Filer som skal installeres i riktig ROS2-sti
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),  # ROS2 krever en ressursfil
        ('share/' + package_name, ['package.xml']),  # Pakke-metadata
        ('share/' + package_name + '/launch', ['launch/error_handler_launch.py']),  # Launch-fil
    ],
    
    install_requires=['setuptools'],  # Python-avhengighet
    zip_safe=True,  # OK for ROS2 (ikke behov for pakkeutpakking)
    
    maintainer='frans',
    maintainer_email='TeemoTeemo123@outlook.com',
    description='TODO: Package description',  # Husk å fylle inn før levering
    license='Apache-2.0',
    
    tests_require=['pytest'],  # Testing

    entry_points={
        'console_scripts': [
            # ROS2-node entry point – kobler ROS-kommando til Python-funksjon
            'error_handler_node = error_handler.error_handler_node:main',
        ],
    },
)

