from setuptools import setup
import os
from glob import glob

package_name = 'cpu_monitor'

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
    maintainer_email='teo.altum.quinque@gmail.com',
    description='Lightweight per-ROS-node CPU/memory profiler for the race stack.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cpu_monitor = cpu_monitor.cpu_monitor:main',
            'plot_cpu_log = cpu_monitor.plot_cpu_log:main',
        ],
    },
)
