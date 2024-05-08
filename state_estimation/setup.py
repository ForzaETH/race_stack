from setuptools import setup
import os
from glob import glob

package_name = 'state_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='forzapblnuc',
    maintainer_email='nicolas.baumann@pbl.ee.ethz.ch',
    description='Handles Localisation and Fusion of sensors for localisation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['carstate_node = state_estimation.carstate_node:main'],
    },
)
