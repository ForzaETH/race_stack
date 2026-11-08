from setuptools import setup
import os
from glob import glob

package_name = 'car_to_car_sync'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maurice Brunner',
    maintainer_email='maurice.brunner@pbl.ee.ethz.ch',
    description='Stream opponent car topics across ROS domains and publish them as ground-truth obstacles',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'domain_bridge = car_to_car_sync.domain_bridge:main',
            'opp_gt_publisher = car_to_car_sync.opp_gt_publisher:main',
        ],
    },
)
