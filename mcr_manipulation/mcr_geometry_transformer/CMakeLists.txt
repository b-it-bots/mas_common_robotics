cmake_minimum_required(VERSION 2.8.3)
project(mcr_geometry_transformer)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    tf
    tf_conversions
    orocos_kdl
    geometry_msgs
    rosunit
)

catkin_python_setup()

catkin_package(
  LIBRARIES geometry_transformer
  CATKIN_DEPENDS
    roscpp
    rospy
    tf
    tf_conversions
    orocos_kdl
    geometry_msgs
    rosunit
)

include_directories(
  common/include
  ros/include
)

add_library(geometry_transformer
  ros/src/geometry_transformer.cpp
  common/src/geometry_transformer_util.cpp
  ${catkin_LIBRARIES}
)


# Unit testing
catkin_add_gtest(geometry_transformer_util_test
  common/test/geometry_transformer_util_test.cpp
)

target_link_libraries(geometry_transformer_util_test
  geometry_transformer
  ${catkin_LIBRARIES}
)

catkin_add_nosetests(common/scripts/test)