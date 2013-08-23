cmake_minimum_required(VERSION 2.8.3)
project(mcr_gradlew)

find_package(catkin REQUIRED)

catkin_package(
  CFG_EXTRAS UseGradle.cmake
)

install(DIRECTORY gradle/
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/"
)

install(FILES gradlew
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/"
)
