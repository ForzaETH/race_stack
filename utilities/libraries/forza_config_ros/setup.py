from setuptools import find_packages, setup

package_name = 'forza_config_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ForzaETH',
    maintainer_email='edoardo.ghignone@pbl.ee.ethz.ch',
    description='ROS 2 layer of forza_config: car config lookup in stack_master, remote parameters',
    license='MIT',
    tests_require=['pytest'],
)
