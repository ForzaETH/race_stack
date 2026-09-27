from setuptools import setup
import os
from glob import glob
from generate_parameter_library_py.setup_helper import generate_parameter_module


package_name = 'local_planner'

# Generates the typed parameters_codegen module from its YAML definition
generate_parameter_module('parameters_codegen', 'local_planner/parameters.yaml')

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
    maintainer_email='maurice.brunner@pbl.ee.ethz.ch',
    description='The local_planner package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_planner = local_planner.local_planner:main', # runs main in local_planner.py
        ],
    },
)
