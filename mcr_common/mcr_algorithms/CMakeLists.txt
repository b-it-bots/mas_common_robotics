cmake_minimum_required(VERSION 2.8.3)
project(mcr_algorithms)
find_package(catkin REQUIRED COMPONENTS libsvm mcr_perception_msgs)
find_package(PCL 1.6 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS common/include/
  LIBRARIES mcr_geometry
#  LIBRARIES mcr_algorithms
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  common/include/
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(mcr_segmentation 
  common/src/segmentation/laserscan_segmentation.cpp
  common/src/segmentation/pointcloud_segmentation.cpp
)

add_library(mcr_geometry 
  common/src/geometry/conversions.cpp
  common/src/geometry/geometric_distances.cpp
  common/src/geometry/geometric_properties.cpp	
)

add_library(mcr_wrapper 
  common/src/wrapper/pcl_wrapper.cpp
)

add_library(mcr_statistics 
  common/src/statistics/means.cpp
  common/src/statistics/minmax.cpp
)

add_library(mcr_projections 
  common/src/projections/pointcloud_projections.cpp
)

add_library(mcr_io 
  common/src/io/file_access.cpp
)

add_library(mcr_machine_learning 
  common/src/machine_learning/random_trees.cpp
  common/src/machine_learning/adaboost.cpp
  common/src/machine_learning/svm.cpp
)

add_library(mcr_data_fitting 
  common/src/data_fitting/circle_2d.cpp
)

add_library(mcr_neural_networks
  common/src/neural_networks/self_organizing_map.cpp
)

add_library(mcr_controller
  common/src/controller/pd_controller.cpp
  common/src/controller/pi_controller.cpp
)

## Declare a cpp executable
# add_executable(mcr_algorithms_node src/mcr_algorithms_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(mcr_algorithms_node mcr_algorithms_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(mcr_algorithms_node
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
# install(TARGETS mcr_algorithms mcr_algorithms_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mcr_algorithms.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
