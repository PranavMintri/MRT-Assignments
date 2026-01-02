from setuptools import find_packages
from setuptools import setup

setup(
    name='aruco_interface',
    version='0.0.0',
    packages=find_packages(
        include=('aruco_interface', 'aruco_interface.*')),
)
