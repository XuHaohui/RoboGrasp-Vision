from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robograsp_vision_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hajimi',
    maintainer_email='user@example.com',
    description='Bringup package for RoboGrasp-Vision',
    license='MIT',
    tests_require=['pytest'],
)
