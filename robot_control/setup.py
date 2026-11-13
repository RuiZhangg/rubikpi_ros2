import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=[
        'setuptools',
        'pyserial>=3.5',
        'numpy>=1.20.0',
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='chl235@ucsd.edu',
    description='Rubik Pi motor control and EKF-SLAM for CSE 276A HW3',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_control = robot_control.motor_control:main',
            'velocity_mapping = robot_control.velocity_mapping:main',
            'hw3_slam = robot_control.hw3_slam_node:main',
            'hw3_controller = robot_control.hw3_controller:main',
        ],
    },
)
