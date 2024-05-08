import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'opponent_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brakestroke',
    maintainer_email='ltognoni@ethz.ch',
    description='Opponent publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collision_detector = opponent_publisher.collision_detector:main',
            'obstacle_publisher = opponent_publisher.obstacle_publisher:main'
        ],
    },
)
