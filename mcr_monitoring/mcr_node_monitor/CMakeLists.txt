cmake_minimum_required(VERSION 2.8.3)
project(mcr_node_monitor)

find_package(catkin REQUIRED COMPONENTS sensor_msgs)

catkin_package(
  CATKIN_DEPENDS sensor_msgs
)

catkin_python_setup()

install(PROGRAMS
    ros/scripts/node_status
    ros/src/mcr_node_monitor/node_status.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
