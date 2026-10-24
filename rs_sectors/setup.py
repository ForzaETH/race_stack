from setuptools import find_packages, setup

package_name = 'rs_sectors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ForzaETH',
    maintainer_email='contact@forzaeth.ch',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sector_tuner = rs_sectors.sector_tuner:main',
            'sector_slicer = rs_sectors.sector_slicer:main',
            'ot_interpolator = rs_sectors.ot_interpolator:main',
        ],
    },
)
