cmake_minimum_required(VERSION 2.8.3)
project(mcr_manipulation_measurers)

find_package(catkin REQUIRED
  COMPONENTS
)

catkin_package(
  CATKIN_DEPENDS
)

install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)