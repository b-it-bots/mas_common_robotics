cmake_minimum_required(VERSION 2.8.3)
project(mcr_aravis_vision)

find_package(catkin REQUIRED
  COMPONENTS
    aravis_vision_library
    dynamic_reconfigure
    image_transport
    mcr_node_diagnostic
    roscpp
    rostest
    sensor_msgs
)
find_package(GTK2 REQUIRED)

generate_dynamic_reconfigure_options(
  ros/config/AravisVision.cfg
)

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure 
    aravis_vision_library
    sensor_msgs
)

include_directories(
  common/include/
  ${catkin_INCLUDE_DIRS}
  ${GTK2_INCLUDE_DIRS}
)

add_library(aravis_wrapper 
  common/src/aravis_wrapper.cpp
)
add_dependencies(aravis_wrapper ${catkin_EXPORTED_TARGETS})

add_executable(aravis_camera_node 
  ros/src/aravis_camera_node.cpp 
)
add_dependencies(aravis_camera_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(aravis_camera_node 
  aravis_wrapper  
  ${catkin_LIBRARIES}
  ${GTK2_LIBRARIES}
)

install(TARGETS aravis_wrapper aravis_wrapper
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS aravis_camera_node aravis_camera_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(DIRECTORY ros/tools/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/tools
)

install(FILES ros/config/stemmer_cvc_eh6500.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config/stemmer_cvc_eh6500.yaml
)

catkin_add_gtest(test_aravis_wrapper 
  common/test/test_aravis_wrapper.cpp
)
target_link_libraries(test_aravis_wrapper 
  aravis_wrapper  
  ${catkin_LIBRARIES}
  ${GTK2_LIBRARIES}
)
