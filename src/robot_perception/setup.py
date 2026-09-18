import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_perception'

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
    description='Lidar sector perception (32-sector SectorScan) from Livox CustomMsg, with RViz markers, for robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_sector_node = robot_perception.lidar_sector_node:main',
            'anomaly_detector_node = robot_perception.anomaly_detector_node:main',
        ],
    },
)
