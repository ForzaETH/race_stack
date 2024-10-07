from setuptools import setup, find_packages

setup(
    name='GraphBasedPlanner',
    version='0.1',
    packages=find_packages(include=['graph_ltpl', 'logs', 'inputs', 'params']),
    install_requires=[
        # Add your dependencies here
    ],
)
