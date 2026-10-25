from setuptools import setup
from generate_parameter_library_py.setup_helper import generate_parameter_module

package_name = 'rs_global_planner'

# Generates the typed parameters_codegen module from its YAML definition
generate_parameter_module('parameters_codegen', 'rs_global_planner/parameters.yaml')

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
    description='The global planner package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_planner = rs_global_planner.planner_node:main',
            'global_plan_server = rs_global_planner.server_node:main'
        ],
    },
)
