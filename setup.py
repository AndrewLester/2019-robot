from setuptools import find_packages, setup

setup(
    name='robot_1418',
    version='1.0.0',
    packages=find_packages(),
    install_requires=['pybind11', 'robotpy-ctre', 'robotpy-navx', 'robotpy-wpilib-utilities', 'pyfrc', 'pynetworktables', 'robotpy-pathfinder'],
    entry_points={
        'robotpy': [
          'generate=robot_src.entry_points.trajectory_generator:TrajectoryGenerator'
        ]
    }
)
