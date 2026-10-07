from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'drone_fsm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ascend',
    maintainer_email='martine.heiestad@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_node = drone_fsm.test_node:main',
            'go_to_node = drone_fsm.go_to_node:main',
            'square = drone_fsm.square:main',
            'circle = drone_fsm.circle:main',
            'heart = drone_fsm.heart:main',
            'six_seven = drone_fsm.six_seven:main',
            'landing_node = drone_fsm.landing_node:main',
            'takeoff_node = drone_fsm.takeoff_node:main',
            'waypoint = drone_fsm.waypoint:main',
            'contoller_sim_node = drone_fsm.controller_sim_node:main',
        ],
    }
)
