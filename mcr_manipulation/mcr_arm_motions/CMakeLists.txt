cmake_minimum_required(VERSION 2.8.3)
project(mcr_arm_motions)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
)

install(PROGRAMS
  ros/scripts/cartesian_motion_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest(ros/launch/cartesian_motion.test)
endif()
