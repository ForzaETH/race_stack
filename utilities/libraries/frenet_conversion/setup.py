from setuptools import find_packages, setup

package_name = 'frenet_conversion'

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
    maintainer='kuehnej',
    maintainer_email='kuehnej@ethz.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frenet_converter_demo = frenet_conversion.frenet_converter_demo_node:main',
        ],
    },
)
