## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

#from distutils.core import setup
#from catkin_pkg.python_setup import generate_distutils_setup

#fetch values from package.xml
#setup_args = generate_distutils_setup(
    #packages=['steering_lookup'],
    #package_dir={'': 'src'},
#)

#setup(**setup_args)
from setuptools import setup
import os
from glob import glob

package_name = 'steering_lookup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'cfg'), glob('cfg/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ForzaETH',
    maintainer_email='nicolas.baumann@pbl.ee.ethz.ch',
    description='The steering_lookup library',
    license='MIT',
    tests_require=['pytest'],
    #entry_points={
        #'console_scripts': [
            #'controller = controller.map_node:main',
        #],
    #},
)