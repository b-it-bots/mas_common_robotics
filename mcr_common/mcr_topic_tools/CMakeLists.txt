cmake_minimum_required(VERSION 2.8.3)

project(mcr_topic_tools)

find_package(catkin
  COMPONENTS
    roscpp
    std_msgs
    nodelet
    pluginlib
    topic_tools
)

catkin_package()

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

### EXECUTABLES
add_library(topic_mux
  ros/src/topic_mux.cpp
)

target_link_libraries(topic_mux
  ${catkin_LIBRARIES}
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()

### INSTALLS
install(TARGETS topic_mux
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
