import os
from glob import glob

from setuptools import setup

package_name = 'longitudinal_sysid_ros'

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
    description='Longitudinal identification experiments that command the VESC directly',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'current_step_node = longitudinal_sysid_ros.current_step_node:main',
        ],
    },
)
