from setuptools import setup
import os
from glob import glob

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ForzaETH',
    maintainer_email='nicolas.baumann@pbl.ee.ethz.ch',
    description='The perception package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracking = perception.tracking:main',
            'detect = perception.detect:main'
        ],
    },
)
