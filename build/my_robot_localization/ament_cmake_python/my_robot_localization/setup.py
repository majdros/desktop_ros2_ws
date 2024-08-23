from setuptools import find_packages
from setuptools import setup

setup(
    name='my_robot_localization',
    version='0.0.0',
    packages=find_packages(
        include=('my_robot_localization', 'my_robot_localization.*')),
)
