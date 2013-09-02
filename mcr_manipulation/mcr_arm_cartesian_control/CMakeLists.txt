cmake_minimum_required(VERSION 2.8.3)
project(mcr_arm_cartesian_control)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs brics_actuator tf urdf kdl_parser mcr_manipulation_utils orocos_kdl)
find_package(eigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS  mcr_manipulation_utils
)

include_directories(
 ${EIGEN_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
 ${PROJECT_SOURCE_DIR}/ros/include ${PROJECT_SOURCE_DIR}/common/include
)

add_executable(arm_cartesian_control
    ros/src/arm_cartesian_control_node.cpp
    common/src/arm_cartesian_control.cpp)
    
target_link_libraries(arm_cartesian_control ${catkin_LIBRARIES})


install(TARGETS arm_cartesian_control arm_cartesian_control
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


