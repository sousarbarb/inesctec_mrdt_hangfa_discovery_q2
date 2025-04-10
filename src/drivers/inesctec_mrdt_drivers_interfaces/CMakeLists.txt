cmake_minimum_required(VERSION 3.5)
project(inesctec_mrdt_drivers_interfaces)



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
  find_package(builtin_interfaces REQUIRED)
  find_package(rosidl_default_generators REQUIRED)



  # Generate interfaces
  rosidl_generate_interfaces(${PROJECT_NAME}
    msg/MotEnc.msg
    msg/MotEncArray.msg
    msg/MotData.msg
    msg/MotDataArray.msg
    msg/MotRef.msg
    msg/MotRefArray.msg
    srv/SetMotorsPWM.srv
    DEPENDENCIES builtin_interfaces
  )



  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()



  ###################################################
  #####  >>> ROBOT OPERATING SYSTEM (ROS) 1 <<< #####
  ###################################################

elseif("$ENV{ROS_VERSION}" STREQUAL "1")

  message(AUTHOR_WARNING "Project version in ROS 1 may be deprecated")

  # Find catkin macros and libraries
  find_package(catkin REQUIRED COMPONENTS
    message_generation
  )

  # Generate messages in the 'msg' folder
  add_message_files(FILES
    MotEnc.msg
    MotEncArrayROS1.msg
    MotData.msg
    MotDataArrayROS1.msg
    MotRef.msg
    MotRefArrayROS1.msg
  )

  # Generate services in the 'srv' folder
  add_service_files(FILES
    SetMotorsPWM.srv
  )

  # Generate added messages and services with any dependencies listed here
  generate_messages()

  # Catkin specific configuration
  catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES inesctec_mrdt_drivers_interfaces
    CATKIN_DEPENDS message_runtime
    # DEPENDS system_lib
  )



  ##########################
  #####  !!! ERROR !!! #####
  ##########################

else()

  message(FATAL_ERROR "No valid version of ROS is installed or environment variables not set")

endif()
