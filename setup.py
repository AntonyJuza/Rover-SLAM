from setuptools import setup
import os
from glob import glob

package_name = 'rover_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'tf_publisher = rover_bringup.tf_publisher:main',
            'compare_rotations = rover_bringup.compare_rotations:main',
            'test_physical_rotation = rover_bringup.test_physical_rotation:main',
            'bno055_node = rover_bringup.bno055_node:main',
        ],
    },
)
