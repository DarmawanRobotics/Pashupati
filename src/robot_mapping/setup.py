import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_mapping'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Agus Darmawan',
    maintainer_email='darmawandeveloper@gmail.com',
    description='FAST-LIO2 mapping bringup and teach-and-repeat path recording with inspection stop points',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_recorder_node = robot_mapping.path_recorder_node:main',
        ],
    },
)
