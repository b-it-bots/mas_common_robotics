cmake_minimum_required(VERSION 2.8.3)
project(mcr_manipulation_monitors)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    rostest
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    mcr_manipulation_msgs
    tf
)

install(PROGRAMS
  ros/scripts/joint_position_monitors_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_add_nosetests(ros/test/joint_position_monitors_test.py)