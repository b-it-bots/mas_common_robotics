cmake_minimum_required(VERSION 2.8.3)
project(mcr_jshop2)

find_package(catkin REQUIRED
  COMPONENTS
    mk
)

catkin_package()

add_custom_target(build_jshop2 ALL
  COMMAND cmake -E chdir "${PROJECT_SOURCE_DIR}" make -f Makefile.git
  
  COMMAND cmake -E copy "${PROJECT_SOURCE_DIR}/build/jshop2/bin.build/JSHOP2.jar" "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/JSHOP2.jar"
)