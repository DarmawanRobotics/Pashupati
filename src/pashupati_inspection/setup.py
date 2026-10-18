from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'pashupati_inspection'

setup(
    name=package_name,
    version='2.0.0',
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
    description='Early anomaly detection on the robot with a local VLM via Ollama',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'anomaly_detector_node = pashupati_inspection.anomaly_detector_node:main',
            'person_detector_node = pashupati_inspection.person_detector_node:main',
        ],
    },
)
