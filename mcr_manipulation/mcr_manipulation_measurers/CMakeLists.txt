cmake_minimum_required(VERSION 2.8.3)
project(mcr_manipulation_measurers)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    rostest
)

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
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)