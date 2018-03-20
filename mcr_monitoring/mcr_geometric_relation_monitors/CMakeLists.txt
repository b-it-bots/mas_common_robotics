cmake_minimum_required(VERSION 2.8.3)
project(mcr_geometric_relation_monitors)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    dynamic_reconfigure
    mcr_manipulation_msgs
    mcr_monitoring_msgs
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  ros/config/ComponentWisePoseErrorMonitor.cfg
)

catkin_package(
  INCLUDE_DIRS
    ros/include
  CATKIN_DEPENDS
    std_msgs
    dynamic_reconfigure
    mcr_manipulation_msgs
    mcr_monitoring_msgs
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY
    ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(DIRECTORY ros/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()
