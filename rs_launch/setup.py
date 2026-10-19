from setuptools import setup

package_name = 'rs_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ForzaETH',
    maintainer_email='contact@forzaeth.ch',
    description="Custom launch actions for the race stack's XML/YAML launch files (e.g. check_args)",
    license='MIT',
    entry_points={
        # exposes the custom launch actions to every XML/YAML launch file once this package is sourced
        'launch.frontend.launch_extension': [
            'rs_launch = rs_launch.launch_extensions',
        ],
    },
)
