import os
import sys
from contextlib import redirect_stdout
from glob import glob

from generate_parameter_library_py.generate_python_module import run as generate_parameter_module
from setuptools import find_packages, setup

package_name = 'prediction'

# Generate the parameter module into the source tree (gitignored), so find_packages picks it up
# (its progress print goes to stderr: colcon parses setup.py's stdout)
here = os.path.dirname(os.path.abspath(__file__))
with redirect_stdout(sys.stderr):
    generate_parameter_module(
        os.path.join(here, 'opponent_prediction', 'parameters_codegen.py'),
        os.path.join(here, 'config', 'opponent_prediction_parameters.yaml'))

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
    maintainer='sai',
    maintainer_email='contact@forzaeth.ch',
    description='Predicts the opponent raceline using Gaussian Processes',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'opponent_prediction = opponent_prediction.opponent_prediction_node:main',
        ],
    },
)
