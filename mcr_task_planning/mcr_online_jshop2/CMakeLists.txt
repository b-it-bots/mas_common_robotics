cmake_minimum_required(VERSION 2.8.3)
project(mcr_online_jshop2)

find_package(Java)
find_package(catkin REQUIRED
  COMPONENTS
    mcr_jshop2
)

include(UseJava)

catkin_package(
  CATKIN_DEPENDS
    mcr_jshop2
)

set(CMAKE_JAVA_INCLUDE_PATH ${mcr_jshop2_JSHOP2_JARS})

add_jar(online_jshop2
  common/de/hbrs/task_planning/online_jshop2/ClassFileManager.java
  common/de/hbrs/task_planning/online_jshop2/Domain.java
  common/de/hbrs/task_planning/online_jshop2/InMemoryJavaFileObject.java
  common/de/hbrs/task_planning/online_jshop2/JavaClassObject.java
  common/de/hbrs/task_planning/online_jshop2/OnlineJSHOP2.java
  common/de/hbrs/task_planning/online_jshop2/Problem.java
)

set(ONLINE_JSHOP2_JAR_PATH ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/online_jshop2.jar)

add_dependencies(online_jshop2 jshop2)

add_custom_target(install_online_jshop2 ALL
  DEPENDS online_jshop2
  
  COMMAND cmake -E copy ${PROJECT_BINARY_DIR}/online_jshop2.jar ${ONLINE_JSHOP2_JAR_PATH}
)
