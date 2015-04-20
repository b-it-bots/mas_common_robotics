cmake_minimum_required(VERSION 2.8.3)
project(mcr_twist_demultiplexer)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  mcr_geometry_transformer
)

catkin_package(
  INCLUDE_DIRS
    ros/include
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    std_msgs
    mcr_geometry_transformer
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(twist_demultiplexer_node
  ros/src/twist_demultiplexer_node.cpp
)

target_link_libraries(twist_demultiplexer_node
  ${catkin_LIBRARIES}
)

install(TARGETS twist_demultiplexer_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
