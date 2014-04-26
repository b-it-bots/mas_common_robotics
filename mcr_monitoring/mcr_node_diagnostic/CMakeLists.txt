cmake_minimum_required(VERSION 2.4.6)
project(mcr_node_diagnostic)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    diagnostic_msgs
    diagnostic_aggregator
    diagnostic_analysis
    pluginlib
)


catkin_package(
  INCLUDE_DIRS
    ros/include
)

catkin_python_setup()

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_library(mcr_node_analyzer
  ros/src/mcr_node_analyzer.cpp
)
  
add_executable(cpp_example
  ros/examples/cpp_example.cpp
)
target_link_libraries(cpp_example
  ${catkin_LIBRARIES}
)


install(TARGETS mcr_node_analyzer
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

