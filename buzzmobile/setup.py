#!/usr/bin/env python
"""Python setup environment for buzzmobile."""

# for downloading the packages needed for this project.
from setuptools import setup

setup(
    name='buzzmobile',
    version='0.3',
    url='https://github.com/gtagency/buzzmobile',
    author='gtagency',
    license='MIT',
    keywords='car cars self-driving lidar gps ros',
    install_requires=[
        'catkin_pkg==0.2.10',
        'googlemaps==2.4.4',
        'matplotlib==1.5.3',
        'numpy==1.11.2',
        'polyline==1.3.1',
        'pylint',
        'pytest==3.0.3',
        'pytest-xdist==1.15.0',
        'pyyaml==3.12',
        'rospkg==1.0.41',
        'ds4drv', # requires apt-get install python-dev
        'empy==3.3.2', # required to make catkin_make work
        'netifaces==0.10.5',  # required for testing
        'psutil==5.1.3',
        ],
)
