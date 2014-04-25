cmake_minimum_required(VERSION 2.8.3)
project(mcr_online_jshop2)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    mcr_gradlew
    mcr_jshop2
    mcr_task_planning_msgs
)

catkin_rosjava_setup(publishMavenJavaPublicationToMavenRepository installApp)

catkin_package(
  CATKIN_DEPENDS
    mcr_jshop2
)

add_executable(jshop2_test
  ros/test/jshop2_test.cpp
)

add_dependencies(jshop2_test
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(jshop2_test
  ${GTEST_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_rostest(ros/test/jshop2.test)
