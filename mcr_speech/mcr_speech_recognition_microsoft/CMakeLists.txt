cmake_minimum_required(VERSION 2.8.3)
project(mcr_speech_recognition_microsoft)

### The code only compiles under windows
if(WIN32)


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

endif(WIN32)
