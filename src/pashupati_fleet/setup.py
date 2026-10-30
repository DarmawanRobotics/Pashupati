from __future__ import annotations

from glob import glob

from setuptools import find_packages, setup

package_name = 'pashupati_fleet'

setup(
    name=package_name,
    version='3.0.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md', 'API.md']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Agus Darmawan',
    maintainer_email='darmawandeveloper@gmail.com',
    description='Robot side of the NETRA command center: telemetry, events, uploads, commands.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fleet_uplink_node = pashupati_fleet.fleet_uplink_node:main',
            'fake_netra = pashupati_fleet.tools.fake_netra:main',
        ],
    },
)
