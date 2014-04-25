cmake_minimum_required(VERSION 2.8.3)
project(mcr_ptu46)


add_action_files(
   DIRECTORY action
   FILES
   PtuGotoPosition.action  
   PtuGotoPositionByName.action  
   PtuReset.action  
   PtuSetVel.action
)

generate_messages(
   DEPENDENCIES
   sensor_msgs
   actionlib_msgs
)

catkin_package(
)

include_directories(
  common/include
  ${catkin_INCLUDE_DIRS}
)


add_library(brsu_ptu46 
	common/src/ptu46_driver.cc
)

add_executable(ptu46_node ros/src/ptu46_node.cc)
target_link_libraries(ptu46_node brsu_ptu46
   ${catkin_LIBRARIES}
)

install(TARGETS ptu46_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ptu46
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_package_example.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)



