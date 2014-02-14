cmake_minimum_required(VERSION 2.8.3)
project(mcr_online_jshop2)

find_package(catkin REQUIRED
  COMPONENTS
    mcr_gradlew
    mcr_jshop2
)

catkin_rosjava_setup(publishMavenJavaPublicationToMavenRepository installApp)

catkin_package(
  CATKIN_DEPENDS
    mcr_jshop2
)