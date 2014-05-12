cmake_minimum_required(VERSION 2.8.3)
project(mcr_kinect_led)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules (LIBUSB REQUIRED libusb-1.0)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
)

add_executable(kinect_led_node
  ros/src/kinect_led_node.cpp
)

add_dependencies(kinect_led_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(kinect_led_node
  ${catkin_LIBRARIES}
  ${LIBUSB_LIBRARIES}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
