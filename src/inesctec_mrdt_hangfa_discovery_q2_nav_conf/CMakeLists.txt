cmake_minimum_required(VERSION 3.5)
project(inesctec_mrdt_hangfa_discovery_q2_nav_conf)



# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



###################################################
#####  >>> ROBOT OPERATING SYSTEM (ROS) 2 <<< #####
###################################################

if("$ENV{ROS_VERSION}" STREQUAL "2")

  # Dependencies
  find_package(ament_cmake REQUIRED)



  # Launch Files Installation Rules
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )



  # Testing
  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()



###################################################
#####  >>> ROBOT OPERATING SYSTEM (ROS) 1 <<< #####
###################################################

elseif("$ENV{ROS_VERSION}" STREQUAL "1")

  message(AUTHOR_WARNING "Project version in ROS 1 may be deprecated")

  # Find catkin macros and libraries
  find_package(catkin REQUIRED)

  # Catkin specific configuration
  catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES sdpo_drivers_interfaces
    # CATKIN_DEPENDS roscpp
    # DEPENDS system_lib
  )

  # Installation rules
  install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )


##########################
#####  !!! ERROR !!! #####
##########################

else()

  message(FATAL_ERROR "No valid version of ROS is installed or environment variables not set")

endif()
