from setuptools import setup
import os
from glob import glob
from generate_parameter_library_py.setup_helper import generate_parameter_module


package_name = 'rs_state_machine'

# Generates the typed parameters_codegen module from its YAML definition
generate_parameter_module('parameters_codegen', 'rs_state_machine/parameters.yaml')

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
    maintainer_email='contact@forzaeth.ch',
    description='Race state machine: decides between raceline tracking, trailing and overtaking',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine = rs_state_machine.state_machine:main',
        ],
    },
)
