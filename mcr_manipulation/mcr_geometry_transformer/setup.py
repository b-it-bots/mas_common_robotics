#!/usr/bin/env python

import distutils.core
import catkin_pkg.python_setup

d = catkin_pkg.python_setup.generate_distutils_setup(
   packages=['geometry_transformer_util', 'mcr_geometry_transformer'],
   package_dir={'geometry_transformer_util': 'common/src/geometry_transformer_util',
                'mcr_geometry_transformer': 'ros/src/mcr_geometry_transformer'}
)

