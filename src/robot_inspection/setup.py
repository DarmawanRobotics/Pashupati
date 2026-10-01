from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robot_inspection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Agus Darmawan',
    maintainer_email='darmawandeveloper@gmail.com',
    description='Early anomaly detection on the robot with a local VLM (Moondream via Ollama)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'anomaly_detector_node = robot_inspection.anomaly_detector_node:main',
        ],
    },
)
