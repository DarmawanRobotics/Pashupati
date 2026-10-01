from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robot_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'PROTOCOL.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Agus Darmawan',
    maintainer_email='darmawandeveloper@gmail.com',
    description='Auto/remote control mux and the UDP teleop gateway (no ROS on the handheld)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'control_mux_node = robot_bridge.control_mux_node:main',
            'teleop_udp_node = robot_bridge.teleop_udp_node:main',
        ],
    },
)
