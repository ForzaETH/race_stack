from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'slam_tuner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'data'), glob('data/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niklas',
    maintainer_email='bastuckn@student.ethz.ch',
    description='A helpful tool for offline tuning of SLAM parameters using rosbags',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_tuner_node = slam_tuner.slam_tuner_node:main'
        ],
    },
)
