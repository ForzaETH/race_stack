from setuptools import setup
from generate_parameter_library_py.setup_helper import generate_parameter_module


package_name = 'rs_mapping'

# Generates the typed parameters_codegen module from its YAML definition
generate_parameter_module('parameters_codegen', 'rs_mapping/parameters.yaml')

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ForzaETH',
    maintainer_email='contact@forzaeth.ch',
    description='Mapping: filters the SLAM map and saves it for the race stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapping_node = rs_mapping.mapping_node:main',
        ],
    },
)
