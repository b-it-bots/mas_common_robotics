cmake_minimum_required(VERSION 2.8.3)
project(mcr_gradlew)

find_package(catkin REQUIRED)

catkin_package(
  CFG_EXTRAS UseGradle.cmake
)
