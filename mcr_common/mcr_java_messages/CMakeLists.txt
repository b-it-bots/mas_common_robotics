cmake_minimum_required(VERSION 2.8.3)
project(mcr_java_messages)

find_package(catkin REQUIRED
  COMPONENTS
    mcr_gradlew
)

catkin_rosjava_setup(publishMavenJavaPublicationToMavenRepository install)

catkin_package()