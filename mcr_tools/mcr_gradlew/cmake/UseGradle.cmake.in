##############################################################################
# Utilities
##############################################################################

set(CATKIN_GLOBAL_MAVEN_DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/maven CACHE PATH "path to which maven artifacts are deployed in your workspace")
set(CATKIN_GLOBAL_GRADLE_DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/gradle CACHE PATH "path to which gradle configuration and artifacts are deployed in your workspace")

# Scans down directories till it finds the gradle wrapper.
# It sets the following variables
# - ${PROJECT_NAME}_gradle_BINARY
macro(find_gradle)
    find_file(${PROJECT_NAME}_gradle_BINARY gradlew
          PATHS 
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${CMAKE_CURRENT_SOURCE_DIR}/..
          ${CMAKE_CURRENT_SOURCE_DIR}/../..
          NO_DEFAULT_PATH
          )
     if(NOT ${PROJECT_NAME}_gradle_BINARY)
         message(FATAL_ERROR "Could not find the gradle wrapper in this directory or below.")
     endif()
endmacro()

# Scans down directories till it finds the gradle project settings. 
# It sets the following variables
# - ${PROJECT_NAME}_gradle_ROOT
macro(find_gradle_repo_root)
    find_file(${PROJECT_NAME}_gradle_SETTINGS settings.gradle
          PATHS 
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${CMAKE_CURRENT_SOURCE_DIR}/..
          ${CMAKE_CURRENT_SOURCE_DIR}/../..
          NO_DEFAULT_PATH
          )
     if(NOT ${PROJECT_NAME}_gradle_SETTINGS)
         message(FATAL_ERROR "Could not find the settings.gradle file in this directory or below.")
     endif()
     get_filename_component(${PROJECT_NAME}_gradle_ROOT ${${PROJECT_NAME}_gradle_SETTINGS} PATH)
endmacro()

# These are used to seed the environment variables if the workspace is
# containing rosjava_build_tools to be built as well. In this situtation
# CATKIN_ENV won't have any configuration, so we need some incoming here.
# Note that we check for the variable existence as well so we don't
# override a user setting.
macro(_rosjava_env)
    set(ROS_MAVEN_DEPLOYMENT_REPOSITORY $ENV{ROS_MAVEN_DEPLOYMENT_REPOSITORY})
    set(ROS_MAVEN_REPOSITORY $ENV{ROS_MAVEN_REPOSITORY})
    if(NOT ROS_MAVEN_DEPLOYMENT_REPOSITORY)
        set(ROSJAVA_ENV "ROS_MAVEN_DEPLOYMENT_REPOSITORY=${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}")
    else()
        set(ROSJAVA_ENV "ROS_MAVEN_DEPLOYMENT_REPOSITORY=${ROS_MAVEN_DEPLOYMENT_REPOSITORY}")
    endif()
    if(NOT ROS_MAVEN_REPOSITORY)
        list(APPEND ROSJAVA_ENV "ROS_MAVEN_REPOSITORY=https://github.com/rosjava/rosjava_mvn_repo/raw/master")
    else()
        set(ROSJAVA_ENV "ROS_MAVEN_REPOSITORY=${ROS_MAVEN_REPOSITORY}")
    endif()
    # The build farm won't let you access /root/.gradle, so redirect it somewhere practical here.
    if(DEFINED CATKIN_BUILD_BINARY_PACKAGE)
      list(APPEND ROSJAVA_ENV "GRADLE_USER_HOME=${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_GRADLE_DESTINATION}")
    endif()
endmacro()

##############################################################################
# RosJava Package
##############################################################################
# Calls the gradle wrapper to compile just the package
# that it is called in with install and installApp targets.
macro(catkin_rosjava_setup)
    _rosjava_env()
    find_gradle()
    if( ${ARGC} EQUAL 0 )
      # Note : COMMAND is a list of variables (semi-colon separated)
      set(gradle_tasks "publishMavenJavaPublicationToMavenRepository") # old targets "install;installApp;uploadArchives"
    else()
      set(gradle_tasks ${ARGV})
    endif()
    add_custom_target(gradle-${PROJECT_NAME}
        ALL
        COMMAND ${ROSJAVA_ENV} ${CATKIN_ENV} "env" "|" "grep" "ROS" 
        COMMAND ${ROSJAVA_ENV} ${CATKIN_ENV} ${${PROJECT_NAME}_gradle_BINARY} ${gradle_tasks} 
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        VERBATIM
    )
    catkin_package_xml()
    foreach(depends in ${${PROJECT_NAME}_BUILD_DEPENDS})
        if(TARGET gradle-${depends})
            #message(STATUS "Adding dependency gradle-${depends}")
            add_dependencies(gradle-${PROJECT_NAME} gradle-${depends})
        endif()
    endforeach()
    if(NOT TARGET gradle-clean)
        add_custom_target(gradle-clean)
    endif()
    add_custom_target(gradle-clean-${PROJECT_NAME}
        COMMAND ${CATKIN_ENV} ${${PROJECT_NAME}_gradle_BINARY} clean
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    add_dependencies(gradle-clean gradle-clean-${PROJECT_NAME})
endmacro()

##############################################################################
# Android Package
##############################################################################
# Calls the gradle wrapper to compile the android package.
# It checks the build type and determines whether it should run
# assembleDebug or assembleRelease
macro(catkin_android_setup)
    _rosjava_env()
    find_gradle()
    if( ${ARGC} EQUAL 0 )
      if(CMAKE_BUILD_TYPE STREQUAL "Release")
        set(gradle_tasks "assembleRelase")
      else()
        set(gradle_tasks "assembleDebug")
      endif()
    else()
      set(gradle_tasks ${ARGV})
    endif()
    add_custom_target(gradle-${PROJECT_NAME}
        ALL
        COMMAND ${ROSJAVA_ENV} ${CATKIN_ENV} ${${PROJECT_NAME}_gradle_BINARY} ${gradle_tasks}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        VERBATIM
    )
    catkin_package_xml()
    foreach(depends in ${${PROJECT_NAME}_BUILD_DEPENDS})
        if(TARGET gradle-${depends})
            #message(STATUS "Adding dependency gradle-${depends}")
            add_dependencies(gradle-${PROJECT_NAME} gradle-${depends})
        endif()
    endforeach()
    if(NOT TARGET gradle-clean)
        add_custom_target(gradle-clean)
    endif()
    add_custom_target(gradle-clean-${PROJECT_NAME}
        COMMAND ${CATKIN_ENV} ${${PROJECT_NAME}_gradle_BINARY} clean
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    add_dependencies(gradle-clean gradle-clean-${PROJECT_NAME})
endmacro()

