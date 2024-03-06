from setuptools import setup, find_packages

setup(
    name='errormetrics',
    version='0.1.0',
    packages=find_packages(include=[
        'errormetrics',
        'errormetrics.*',
        'csv_io.py',
        'plot_things.py',
        'utils.py'
        ]),
    install_requires=[
        'numpy',
        'matplotlib',
    ],
)