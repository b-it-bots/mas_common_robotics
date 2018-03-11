#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mcr_saved_trajectories', 'mcr_saved_trajectories_ros'],
    package_dir={'mcr_saved_trajectories': 'common/src/mcr_saved_trajectories',
        'mcr_saved_trajectories_ros': 'ros/src/mcr_saved_trajectories_ros'}
)

setup(**d)
