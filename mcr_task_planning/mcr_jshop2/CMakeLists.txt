cmake_minimum_required(VERSION 2.8.3)
project(mcr_jshop2)

find_package(catkin REQUIRED
  COMPONENTS
    mcr_gradlew
)

include(ExternalProject)

ExternalProject_Add(jshop2
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/jshop2
  GIT_REPOSITORY https://github.com/mas-group/jshop2.git
  GIT_TAG master
  CONFIGURE_COMMAND ""
  UPDATE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

# the sources are checked out to the build folder and the gradle build is also
# running there, so copy over the build script
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/build.gradle
  ${CMAKE_CURRENT_BINARY_DIR}/build.gradle
)

# building with gradle's ros plugins requires a package.xml next to the
# build.gradle file
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
  ${CMAKE_CURRENT_BINARY_DIR}/package.xml
)

catkin_rosjava_setup_dir(${CMAKE_CURRENT_BINARY_DIR} publishMavenJavaPublicationToMavenRepository)

# "gradle-${PROJECT_NAME}" is a target provided by "catkin_rosjava_setup_dir"
# (see above) for building jshop2
# adding this dependency makes sure that the jshop2 code is downloaded before
# the build
add_dependencies(gradle-${PROJECT_NAME} jshop2)

catkin_package()