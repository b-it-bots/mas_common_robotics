cmake_minimum_required(VERSION 2.8.3)
project(mcr_node_monitor)

find_package(catkin REQUIRED COMPONENTS sensor_msgs)

catkin_package(
  CATKIN_DEPENDS sensor_msgs
)

catkin_python_setup()

install(PROGRAMS
    ros/scripts/mcr_node_status
    ros/src/mcr_node_monitor/mcr_node_status.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# install(DIRECTORY
#    ros/scripts
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY
#    ros/src
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

