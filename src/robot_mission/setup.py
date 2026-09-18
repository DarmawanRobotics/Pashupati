import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_mission'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'trees'), glob('robot_mission/trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Agus Darmawan',
    maintainer_email='darmawandeveloper@gmail.com',
    description='XML-defined behavior-tree mission orchestration for robot_navigation, using a NavigateRoute action',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_node = robot_mission.mission_node:main',
        ],
    },
)
