cmake_minimum_required(VERSION 2.8.3)
project(mcr_kinect_led)

find_package(catkin REQUIRED roscpp std_msgs)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules (LIBUSB REQUIRED libusb-1.0)

catkin_package()

include_directories(
  ${LIBUSB_INCLUDE_DIRS}
)

link_libraries(
  ${LIBUSB_LIBRARY_DIRS}
)

add_executable(kinect_led_node
  ros/src/kinect_led_node.cpp
)

target_link_libraries(kinect_led_node
  ${catkin_LIBRARIES}
  usb-1.0
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
