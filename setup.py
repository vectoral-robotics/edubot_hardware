from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'omnibot_hardware'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        # ROS package index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Optional: install any launch files for direct use
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'pyserial',  # rosdep key: python3-serial
    ],
    zip_safe=True,
    maintainer='Stefan Siegler',
    maintainer_email='dev@siegler.cône',
    description='ROS 2 Hardware and Simulation Backend for the Omnidirectional Robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_node = omnibot_hardware.hardware_node:main',
        ],
    },
)
