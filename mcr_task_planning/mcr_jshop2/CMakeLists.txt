cmake_minimum_required(VERSION 2.8.3)
project(mcr_jshop2)

find_package(catkin REQUIRED)

include(ExternalProject)

# ensure that package destination variables are defined
catkin_destinations()

set(JSHOP2_JAR_PATH ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/JSHOP2.jar)
set(ANTLR_JAR_PATH ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/antlr.jar)
set(BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/jshop2/src/jshop2)

ExternalProject_Add(jshop2
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/jshop2
  GIT_REPOSITORY https://github.com/mas-group/jshop2.git
  GIT_TAG master
  CONFIGURE_COMMAND install -dm755 ${BUILD_DIR}/bin.build
  BUILD_COMMAND CLASSPATH=${BUILD_DIR}:${BUILD_DIR}/antlr.jar make -C ${BUILD_DIR}
  INSTALL_COMMAND ""
)

catkin_package(
  CFG_EXTRAS UseJSHOP2.cmake
)

add_custom_target(copy_jshop2 ALL
  DEPENDS jshop2
  
  COMMAND cmake -E copy ${BUILD_DIR}/bin.build/JSHOP2.jar ${JSHOP2_JAR_PATH}
  COMMAND cmake -E copy ${BUILD_DIR}/antlr.jar ${ANTLR_JAR_PATH}
)