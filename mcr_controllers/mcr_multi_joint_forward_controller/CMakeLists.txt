cmake_minimum_required(VERSION 2.8.3)
project(mcr_multi_joint_forward_controller)

find_package(catkin REQUIRED
  COMPONENTS
    controller_interface
    hardware_interface
    brics_actuator
    rostest
)

catkin_package(
  INCLUDE_DIRS ros/include
  CATKIN_DEPENDS
    controller_interface
    hardware_interface
    brics_actuator
    rostest
)

# build
include_directories(ros/include)

# install
install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# test
add_rostest_gtest(multi_joint_forward_controller_test
  ros/test/multi_joint_forward_controller.test
  ros/test/multi_joint_forward_controller_test.cpp
)

target_link_libraries(multi_joint_forward_controller_test
  ${catkin_LIBRARIES}
)