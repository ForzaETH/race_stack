from setuptools import find_packages, setup

package_name = 'pbl_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pydantic'],
    zip_safe=True,
    maintainer='maubrunn',
    maintainer_email='maurice.brunner@pbl.ee.ethz.ch',
    description='Pydantic-validated config loaders (car, pacejka tire, linear tire) for the race stack',
    license='MIT',
    tests_require=['pytest'],
)
