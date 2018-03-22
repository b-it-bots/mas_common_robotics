^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mcr_interpolation_planner_moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* removing unused header in interpolation_planner_context.h
* boost::shared_ptr to std::shared_ptr in interpolation_planner_context.h modified... compiles but not tested
* add scoped_ptr to headers
* removing CATKIN_IGNORE
* CMake requires c++11
* Merge branch 'kinetic' of mas.b-it-center.de:mas-group/mas_common_robotics into kinetic
* Add CATKIN_IGNORE files to packages that don't build
* Merge branch 'indigo' of mas.b-it-center.de:mas-group/mas_common_robotics into mercury_planner
* [interpolation planner] add missing dependency
* [interpolation planner] remove link directories
* Merge branch 'hydro' into indigo
* [interpolation planner] remove unnecessary build dependency
* update dependencies
* Merge branch 'hydro' into indigo
  Conflicts:
  mcr_controllers/mcr_trajectory_controller/ros/include/mcr_trajectory_controller/hardware_interface_adapter.h
  mcr_controllers/mcr_trajectory_controller/ros/include/trajectory_interface/pos_vel_acc_state.h
  mcr_controllers/mcr_trajectory_controller/ros/include/trajectory_interface/quintic_spline_segment.h
  mcr_controllers/mcr_trajectory_controller/ros/include/trajectory_interface/trajectory_interface.h
  mcr_controllers/mcr_trajectory_controller/ros/src/trajectory_controller.cpp
* [formatting] replace tabs by spaces, indentation and ending spaces
* Merge branch 'hydro' into indigo
* Merge branch 'moveit_interpolation_planner_plugin_revised_55' into 'hydro'
  Moveit interpolation planner plugin(Revised pull request#55)
  - Integrated changes that were suggested in pull request#55.
  See merge request !69
* refactor code to follow the coding convention suggsted by rosling(spaces, line length)
* add LIBRARIES to catkin_package
  to expose package libraries to other packages
* Refactor install function to specifiy include  directory of package.
* change license: BSD-> GPLv3
* add interpolation planner plugin for Moveit! planning pipeline.
* Contributors: Alex Mitrevski, Argentina Ortega Sainz, Frederik Hegger, Oscar Lima, demo@c069-workstation-1, jcmayoral, shehzad ahmed
