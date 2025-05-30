from setuptools import find_packages, setup

package_name = 'movement_controller'

setup(
    name=package_name,  # Navnet på pakken
    version='0.0.0',  # Versjonsnummer
    packages=find_packages(exclude=['test']),  # Automatisk inkluderer alle underpakker unntatt 'test'
    data_files=[
        # Gjør at ament kan indeksere pakken
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Installerer package.xml
        ('share/' + package_name, ['package.xml']),
        # Inkluderer launch-filene
        ('share/' + package_name + '/launch', ['launch/movement_controller_launch.py']),
    ],
    install_requires=['setuptools'],  # Ekstern avhengighet (Python-standard)
    zip_safe=True,  # Tillater at pakken pakkes som .egg (ingen behov for å være unzip'et)
    maintainer='frans',
    maintainer_email='TeemoTeemo123@outlook.com',
    description='TODO: Package description',  # Legg inn en beskrivelse senere
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Definerer navnet på kommandolinjeverktøyet og peker til main-funksjonen
            'movement_controller_node = movement_controller.movement_controller_node:main',
        ],
    },
)
