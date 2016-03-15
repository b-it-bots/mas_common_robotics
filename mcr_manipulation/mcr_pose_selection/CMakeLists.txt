cmake_minimum_required(VERSION 2.8.3)
project(mcr_pose_selection)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    actionlib
    geometry_msgs
    moveit_msgs
    brics_actuator
    moveit_commander
)


### TESTS
roslint_python(
  ros/scripts/reachability_pose_selector_node
  ros/scripts/reachability_pose_selector_trigger_node
)

install(PROGRAMS
  ros/scripts/reachability_pose_selector_node
  ros/scripts/reachability_pose_selector_trigger_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
