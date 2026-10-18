from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'pashupati_perception'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Agus Darmawan',
    maintainer_email='darmawandeveloper@gmail.com',
    description='Camera person detector (YOLO) for the handheld and NETRA',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_detector_node = pashupati_perception.person_detector_node:main',
        ],
    },
)
