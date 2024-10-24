from setuptools import setup
import os
from glob import glob

package_name = 'stack_master'


#handle the maps subfolders
map_subfolders = []
for map_dir in os.listdir('maps'):
    map_subfolders.append((os.path.join('share', package_name, 'maps', map_dir), glob('maps/{}/*'.format(map_dir), recursive=True)))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch', 'subsystems'), glob(os.path.join('launch', 'subsystems', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'NUC2'), glob(os.path.join('config', 'NUC2', '*/*'), recursive=True)),
        (os.path.join('share', package_name, 'config', 'NUC2'), glob(os.path.join('config', 'NUC2', '*.yaml'))),
        (os.path.join('share', package_name, 'config', 'NUC5'), glob(os.path.join('config', 'NUC5', '*/*'), recursive=True)),
        (os.path.join('share', package_name, 'config', 'NUC5'), glob(os.path.join('config', 'NUC5', '*.yaml'))),
        (os.path.join('share', package_name, 'config', 'NUC6'), glob(os.path.join('config', 'NUC6', '*/*'), recursive=True)),
        (os.path.join('share', package_name, 'config', 'NUC7'), glob(os.path.join('config', 'NUC7', '*.yaml'))),(os.path.join('share', package_name, 'config', 'NUC7'), glob(os.path.join('config', 'NUC7', '*/*'), recursive=True)),
        (os.path.join('share', package_name, 'config', 'NUC7'), glob(os.path.join('config', 'NUC7', '*.yaml'))),
        (os.path.join('share', package_name, 'config', 'SIM'), glob(os.path.join('config', 'SIM', '*.*'))),
        (os.path.join('share', package_name, 'config', 'global_planner'), glob(os.path.join('config', 'global_planner', '*.*'))),
        (os.path.join('share', package_name, 'config', 'global_planner', 'veh_dyn_info'), glob(os.path.join('config', 'global_planner', 'veh_dyn_info', '*.csv'))),
        *map_subfolders,
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ForzaETH',
    maintainer_email='nicolas.baumann@pbl.ee.ethz.ch',
    description='The package to rule them all: Launch files and Configs',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_parameter_node = stack_master.global_parameter_node:main',
        ],
    },
)
