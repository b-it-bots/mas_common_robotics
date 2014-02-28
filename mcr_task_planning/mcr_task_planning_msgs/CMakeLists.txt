cmake_minimum_required(VERSION 2.8.3)
project(mcr_task_planning_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
    actionlib_msgs
)

add_message_files(
  FILES
    Action.msg
    Atom.msg
    Plan.msg
    State.msg
    Task.msg
)

add_action_files(
  DIRECTORY action
  FILES
    HtnPlanning.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    actionlib_msgs
)