from setuptools import setup

package_name = 'global_planner'

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
    maintainer_email='nicolas.baumann@pbl.ee.ethz.ch',
    description='The global planner package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_planner = global_planner.global_planner_node:main',
            'global_trajectory_publisher = global_planner.global_trajectory_publisher:main'
        ],
    },
)
