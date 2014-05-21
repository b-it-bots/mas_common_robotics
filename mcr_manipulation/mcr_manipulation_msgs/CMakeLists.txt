cmake_minimum_required(VERSION 2.8.3)
project(mcr_manipulation_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
)

add_message_files(
  FILES
    SphericalSamplerParameters.msg
)

generate_messages(
  DEPENDENCIES
    mcr_common_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
)