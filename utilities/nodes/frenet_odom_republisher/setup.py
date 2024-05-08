from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'frenet_odom_republisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nadine Imholz',
    maintainer_email='nimholz@ethz.ch',
    description='Republishing odometry in frenet frame',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frenet_odom_republisher_node = frenet_odom_republisher.frenet_odom_republisher_node:main'
        ],
    },
)
