cmake_minimum_required(VERSION 2.8.3)
project(mcr_pose_generator)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    rostest
    tf
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    numpy
    rospy
    std_msgs
    geometry_msgs
    mcr_manipulation_msgs
    tf
)

install(PROGRAMS
  ros/scripts/pose_generator_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(ros/launch)
  add_rostest(ros/launch/pose_generator.test)
  catkin_add_nosetests(ros/test/pose_generator_unittest.py)
  catkin_add_nosetests(common/test/transformation_unit_test.py)
endif()