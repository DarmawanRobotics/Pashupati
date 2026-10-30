from __future__ import annotations

from setuptools import find_packages, setup

package_name = 'pashupati_utils'

setup(
    name=package_name,
    version='3.0.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Agus Darmawan',
    maintainer_email='darmawandeveloper@gmail.com',
    description='Shared helpers of the Pashupati packages: typed status to wire values.',
    license='Apache-2.0',
)
