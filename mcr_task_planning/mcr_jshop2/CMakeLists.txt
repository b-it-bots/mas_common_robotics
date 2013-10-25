cmake_minimum_required(VERSION 2.8.3)
project(mcr_jshop2)

find_package(catkin REQUIRED
  COMPONENTS
    mk
)

catkin_package()


# IMPORTANT: If one of these paths is changed, also update the
# cmake/UseJSHOP2.cmake.in file!!! The ${CATKIN_PACKAGE_SHARE_DESTINATION} is
# only defined after the catkin_package macro, so these paths can not be moved
# before the execution of this macro!!!
set(JSHOP2_JAR_PATH "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/JSHOP2.jar")
set(ANTLR_JAR_PATH "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/antlr.jar")


add_custom_target(build_jshop2 ALL
  COMMAND cmake -E chdir "${PROJECT_SOURCE_DIR}" make -f Makefile.git
  
  COMMAND cmake -E copy "${PROJECT_SOURCE_DIR}/build/jshop2/bin.build/JSHOP2.jar" "${JSHOP2_JAR_PATH}"
  COMMAND cmake -E copy "${PROJECT_SOURCE_DIR}/build/jshop2/antlr.jar" "${ANTLR_JAR_PATH}"
)