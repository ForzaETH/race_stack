from setuptools import setup

package_name = 'rs_lap_analysis'

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
    description='Lap and sector statistics of the race car',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lap_analyser = rs_lap_analysis.lap_analyser:main',
        ],
    },
)
