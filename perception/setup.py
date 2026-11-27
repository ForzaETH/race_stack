from setuptools import setup
import os
from contextlib import redirect_stdout
from glob import glob

from generate_parameter_library_py.generate_python_module import run as generate_parameter_module

package_name = 'perception'

# Generate the parameter modules of the new nodes from their YAML definitions
here = os.path.dirname(os.path.realpath(__file__))
with open(os.devnull, 'w') as devnull, redirect_stdout(devnull):
    for node in ('obstacle_detection', 'opponent_tracking'):
        generate_parameter_module(
            os.path.join(here, package_name, f'{node}_parameters_codegen.py'),
            os.path.join(here, 'config', f'{node}_parameters.yaml'))

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
    maintainer_email='contact@forzaeth.ch',
    description='The perception package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'obstacle_detection = perception.obstacle_detection_node:main',
            'opponent_tracking = perception.opponent_tracking_node:main'
        ],
    },
)
