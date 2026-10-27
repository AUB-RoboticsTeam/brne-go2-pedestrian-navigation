from setuptools import find_packages, setup

import os
from glob import glob


package_name = 'pedestrian_tracker'


setup(
    name=package_name,

    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[

        (
            'share/ament_index/resource_index/packages',
            [
                'resource/' + package_name
            ],
        ),

        (
            'share/' + package_name,
            [
                'package.xml'
            ],
        ),

        # Install launch files
        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),

        # Install RViz configuration
        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('config/*.rviz')
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='user',

    maintainer_email='user@example.com',

    description=(
        'Pedestrian position and velocity tracking'
    ),

    license='Apache-2.0',

    entry_points={

        'console_scripts': [

            'pedestrian_state_node = '
            'pedestrian_tracker.pedestrian_state_node:main',

            'robot_pose_tf = '
            'pedestrian_tracker.robot_pose_tf:main',

            'camera_link_marker = '
            'pedestrian_tracker.camera_link_marker:main',

            'unitree_cmd_vel_bridge = '
            'pedestrian_tracker.unitree_cmd_vel_bridge:main',
        ],
    },
)
