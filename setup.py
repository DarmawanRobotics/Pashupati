import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'moondream_anomaly_detector'

setup(
    name=package_name,
    version='0.1.0',
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
    description='Moondream VLM-based anomaly detection (trash, spill, fallen person) over a ROS2 image topic, via Ollama',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'anomaly_detector_node = moondream_anomaly_detector.anomaly_detector_node:main',
        ],
    },
)