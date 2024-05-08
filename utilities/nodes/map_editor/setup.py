from setuptools import setup
import os
from glob import glob

package_name = 'map_editor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ForzaETH',
    maintainer_email='tialim@student.ethz.ch',
    description='The map editor package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)
