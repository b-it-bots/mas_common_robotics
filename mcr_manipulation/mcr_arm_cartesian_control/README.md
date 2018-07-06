This packege no longer work with the default KDL version in ros-kinetic. 
## Steps to build.
1. Uninstall ros-kinetic-orocos-kdl
2. Clone orocos-kdl  (git@github.com:orocos/orocos_kinematics_dynamics.git)
3. Clone kdl-parser and checkout to kinetic-devel branch  (https://github.com/ros/kdl_parser)
4. Build this package
5. Install all the libraries uninstalledd in step 1
6. Build the whole workspace
