cmake_minimum_required(VERSION 2.8.3)
project(mcr_geometric_relation_monitors)

find_package(catkin REQUIRED
  COMPONENTS
    roslint
    rospy
    std_msgs
    mcr_manipulation_msgs
    mcr_monitoring_msgs
)

catkin_python_setup()

roslint_python()

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    mcr_manipulation_msgs
    mcr_monitoring_msgs
)
