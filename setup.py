from setuptools import setup
import os
from glob import glob

package_name = 'skyjackers_drone_network'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nolan Stilwell',
    maintainer_email='nolanstilwell132@gmail.com',
    description='Drone network package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_network = skyjackers_drone_network.drone_network:main',
            'simple_monitor = skyjackers_drone_network.simple_monitor:main',
        ],
    },
)
