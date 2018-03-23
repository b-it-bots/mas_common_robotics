#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mcr_geometric_relation_monitors_ros'],
   package_dir={'mcr_geometric_relation_monitors_ros': 'ros/src/mcr_geometric_relation_monitors_ros'}
)

setup(**d)
