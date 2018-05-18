cmake_minimum_required(VERSION 2.8.3)
project(mcr_manipulation_pose_selector)

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
  ros/src/mcr_manipulation_pose_selector/reachability_pose_selector.py
  ros/src/mcr_manipulation_pose_selector/reachability_pose_selector_utils.py
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(ros/test/reachability_pose_selector_unittest.py)
endif()
