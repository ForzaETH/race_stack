import os
from glob import glob

from setuptools import setup

package_name = 'lateral_sysid_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ForzaETH',
    maintainer_email='edoardo.ghignone@pbl.ee.ethz.ch',
    description='ROS 2 nodes and tools around on_track_sysid',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'on_track_node = lateral_sysid_ros.on_track_node:main',
            'steady_state_from_bag = lateral_sysid_ros.steady_state_from_bag:main',
            'generate_lut = lateral_sysid_ros.generate_lut:main',
            'steering_ramp_node = lateral_sysid_ros.steering_ramp_node:main',
            'pacejka_visualizer = lateral_sysid_ros.pacejka_visualizer:main',
            'max_steering_from_bag = lateral_sysid_ros.max_steering_from_bag:main',
        ],
    },
)
