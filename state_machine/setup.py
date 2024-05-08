from setuptools import setup
import os
from glob import glob


package_name = 'state_machine'

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
    maintainer='ForzaETH',
    maintainer_email='nicolas.baumann@pbl.ee.ethz.ch',
    description='The state_machine package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine = state_machine.state_machine:main', # runs main in state_machine.py
        ],
    },
)
