^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mcr_multi_joint_forward_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge branch 'kinetic' of mas.b-it-center.de:mas-group/mas_common_robotics into kinetic
* Add CATKIN_IGNORE files to packages that don't build
* Merge branch 'indigo' of mas.b-it-center.de:mas-group/mas_common_robotics into mercury_planner
* add support for plain float arrays
* [multi joint forward controller] fix test
* [multi joint forward controller] use latched publisher
* [multi joint forwarder controller] use sleep in test to wait for result
* [multi joint forward controller] increase loop limit
* [multi joint forward controller] increase waiting duration to make the test pass
* fix formatting
* Merge branch 'hydro' into indigo
  Conflicts:
  mcr_controllers/mcr_trajectory_controller/ros/include/mcr_trajectory_controller/hardware_interface_adapter.h
  mcr_controllers/mcr_trajectory_controller/ros/include/trajectory_interface/pos_vel_acc_state.h
  mcr_controllers/mcr_trajectory_controller/ros/include/trajectory_interface/quintic_spline_segment.h
  mcr_controllers/mcr_trajectory_controller/ros/include/trajectory_interface/trajectory_interface.h
  mcr_controllers/mcr_trajectory_controller/ros/src/trajectory_controller.cpp
* [formatting] replace tabs by spaces, indentation and ending spaces
* Merge branch 'hydro' of mas.b-it-center.de:mas-group/mas_common_robotics into moveit_interpolation_planner_plugin_revised_55
* Merge branch 'hydro' into indigo
  Conflicts:
  mcr_navigation/mcr_recovery_behaviors/package.xml
* [lint] update dependencies
  to pass the catkin lint test
* Merge branch 'hydro' into indigo
  Conflicts:
  repository.debs
* Merge branch 'hydro' of git-pi:b-it-bots/mas_common_robotics into hydro
* Merge branch 'hydro' into twist_demux
* Merge branch 'hydro' into image_transform
* Merge branch 'hydro' of mas.b-it-center.de:mas-group/mas_common_robotics into hydro
* change from unit test to ros integration test
* Merge branch 'hydro' of github.com:mas-group/mas_common_robotics into hydro
  Conflicts:
  .gitignore
* add missing installs
* export library
* add missing include
* format cmake file
* fix a buffer overflow when the number of values in the incoming command is greater than the buffer size
* wait longer for the message to be received in order to decrease the chances that the test fails
* adapt the topic name to command_vel also in the test
* Merge branch 'hydro' of github.com:mas-group/mas_common_robotics into hydro
  Conflicts:
  mcr_environments/mcr_gazebo_worlds/ros/launch/brsu-c025.launch
* use the command_vel topic (brics_actuator messages) instead of the command topic (trajectory messages)
* offer the actionlib ros control plugin to other packages
* build an actionlib library with the controllers
* build a library from the header files which can be used as ros controller
* don't install the header files
* initial version of the multi joint forward command controller interface
* Contributors: Alex Mitrevski, Argentina Ortega Sainz, Ashok Meenakshi Sundaram, Frederik Hegger, Oscar Lima, Shehzad Ahmed, Sven Schneider, demo@cob3-1-pc1, shehzad ahmed
