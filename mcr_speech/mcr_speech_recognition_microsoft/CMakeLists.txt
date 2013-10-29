cmake_minimum_required(VERSION 2.8.3)
project(mcr_speech_recognition_microsoft)


#set(CMAKE_CXX_FLAGS "/DWIN32 /D_WINDOWS /W3 /EHa /GR")
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/bin)
#set(SPEECH_RECOGNITION_PATH ${CMAKE_CURRENT_SOURCE_DIR}/SpeechRecognition)
#add_definitions( "/W3 /D_CRT_SECURE_NO_WARNINGS /wd4251 /wd4275 /wd4099  /DROS_BUILD_SHARED_LIBS=1" ) #suppress annoying warnings
#add_definitions( "/W3 /D_CRT_SECURE_NO_WARNINGS /wd4251 /wd4275 /wd4099" ) #suppress annoying warnings

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mcr_speech_recognition_microsoft
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  common/include
  #${CMAKE_SOURCE_DIR}/../../brsu_common/brsu_srvs/srv_gen/cpp/include/brsu_srvs
  #${CMAKE_SOURCE_DIR}/../../brsu_common/brsu_msgs/msg_gen/cpp/include/brsu_msgs
  #${SPEECH_RECOGNITION_PATH}/include
  #${CMAKE_SOURCE_DIR}/../brsu_speech_common/include/
  #C:/opt/rosdeps/fuerte/x86/include
  #C:/opt/ros/fuerte/x86/include
	#C:/ros_win/include
	#C:/external_libs/boost_1_44_0
)

add_library(speech_recognition_microsoft
  common/src/speech_recognition.cpp
  common/src/grammar_parser.cpp
)

add_library(speech_recognition_tools
  common/src/string_operations.cpp
  common/src/config_file_reader.cpp
)


#link_directories (
#	C:/opt/rosdeps/fuerte/x86/lib
#	C:/opt/ros/fuerte/x86/lib;
#	C:/ros_win/lib
#	C:/external_libs/boost_1_44_0/stage/lib
#)

#link_libraries(
#	SpeechRecognition
#	roscpp.lib
#	roscpp_serialization.lib
#	rosconsole.lib
#	rostime.lib
#  SpeechRecognition
#	cpp_common
#	log4cxx
#	ros
#	rosconsole
#	roscpp_serialization
#	roslib
#	rostime
#	XmlRpc
#)


add_executable(speech_recognition_node
  speech_recognition_node.cpp
)
target_link_libraries(speech_recognition_node
  speech_recognition_tools
)

add_executable(speech_recognition_test 
  speech_recognition_test.cpp
)
target_link_libraries(speech_recognition_test
  speech_recognition_tools
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(mcr_speech_recognition_microsoft
#   src/${PROJECT_NAME}/mcr_speech_recognition_microsoft.cpp
# )

## Declare a cpp executable
# add_executable(mcr_speech_recognition_microsoft_node src/mcr_speech_recognition_microsoft_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(mcr_speech_recognition_microsoft_node mcr_speech_recognition_microsoft_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(mcr_speech_recognition_microsoft_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS mcr_speech_recognition_microsoft mcr_speech_recognition_microsoft_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mcr_speech_recognition_microsoft.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
