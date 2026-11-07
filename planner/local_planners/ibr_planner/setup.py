from setuptools import find_packages, setup
import os
from glob import glob 


package_name = 'ibr_planner'

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
    maintainer='ychuang',
    maintainer_email='ychuang@ethz.ch',
    description='Iterated best response local planner',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ibr_planner = ibr_planner.ibr_planner:main',
            'mirror_planner = ibr_planner.mirror_planner:main',
        ],
    },
)
