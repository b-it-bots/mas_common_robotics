cmake_minimum_required(VERSION 2.8.3)
project(mcr_speech_synthesis)

### The code only compiles under windows
if(WIN32)

set(CMAKE_CXX_FLAGS "/DWIN32 /D_WINDOWS /W3 /EHa /GR")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/bin) #executables
add_definitions( "/W3 /D_CRT_SECURE_NO_WARNINGS /wd4251 /wd4275 /wd4099  /DROS_BUILD_SHARED_LIBS=1" ) #suppress annoying warnings

find_package(catkin REQUIRED)

catkin_package()

include_directories(
  C:/opt/rosdeps/fuerte/x86/include
  C:/opt/ros/fuerte/x86/include
  ${CMAKE_SOURCE_DIR}/../../brsu_common/brsu_msgs/msg_gen/cpp/include/brsu_msgs
  ${CMAKE_SOURCE_DIR}/../../brsu_common/brsu_srvs/srv_gen/cpp/include/brsu_srvs
  ${CMAKE_SOURCE_DIR}/../brsu_speech_common/include
)

link_directories(
	C:/opt/rosdeps/fuerte/x86/lib
	C:/opt/ros/fuerte/x86/lib;
)

link_libraries(
	roscpp.lib
	roscpp_serialization.lib
	rosconsole.lib
	rostime.lib
)

add_executable(speech_synthesis_node 
  ros/src/speech_synthesis_node.cpp
  {CMAKE_SOURCE_DIR}/../brsu_speech_common/src/StringOperations.cpp
  ${CMAKE_SOURCE_DIR}/../brsu_speech_common/include/StringOperations.h
  ${CMAKE_SOURCE_DIR}/../brsu_speech_common/src/ConfigFileReader.cpp
  ${CMAKE_SOURCE_DIR}/../brsu_speech_common/include/ConfigFileReader.h
)

endif(WIN32)
