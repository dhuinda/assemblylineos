from setuptools import setup
import os
from glob import glob

package_name = 'assembly_line_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'templates'), glob('templates/*')),
        (os.path.join('share', package_name, 'static'), glob('static/**/*', recursive=True)),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['flask', 'flask-cors', 'pyserial'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Web interface for controlling motors and relays via ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_interface = assembly_line_control.web_interface:main',
            'motor_controller = assembly_line_control.motor_controller:main',
            'relay_controller = assembly_line_control.relay_controller:main',
            'sensor_controller = assembly_line_control.sensor_controller:main',
        ],
    },
)

