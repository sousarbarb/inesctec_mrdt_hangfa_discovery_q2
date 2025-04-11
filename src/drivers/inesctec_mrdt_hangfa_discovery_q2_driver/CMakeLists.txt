cmake_minimum_required(VERSION 3.5)
project(inesctec_mrdt_hangfa_discovery_q2_driver)



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
  find_package(rclcpp REQUIRED)
  find_package(inesctec_mrdt_drivers_interfaces REQUIRED)
  find_package(inesctec_mrdt_serial_port REQUIRED)
  find_package(serial_communication_channels REQUIRED)



  # Node Executable
  add_executable(${PROJECT_NAME}
    src/${PROJECT_NAME}/InesctecMrdtHangfaDiscoveryQ2.cpp
    src/${PROJECT_NAME}/InesctecMrdtHangfaDiscoveryQ2DriverROS2.cpp
    src/${PROJECT_NAME}/SerialChannelsConfig.cpp
    src/main_ros2.cpp
  )

  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    inesctec_mrdt_drivers_interfaces
    inesctec_mrdt_serial_port
    serial_communication_channels
  )



  # Node Executable Installation Rules
  install(
    TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )



  # Launch Files Installation Rules
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )



  # Default Config Installation Rules
  # install(
  #   DIRECTORY config
  #   DESTINATION share/${PROJECT_NAME}
  # )



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
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    inesctec_mrdt_drivers_interfaces
    inesctec_mrdt_serial_port
    serial_communication_channels
  )

  # Catkin specific configuration
  catkin_package(
    INCLUDE_DIRS include
    # LIBRARIES inesctec_mrdt_drivers_interfaces
    CATKIN_DEPENDS
      roscpp
      inesctec_mrdt_drivers_interfaces
      inesctec_mrdt_serial_port
      serial_communication_channels
    # DEPENDS system_lib
  )

  # Build
  # - additional header files
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  # - declare a C++ executable
  add_executable(${PROJECT_NAME}
    src/${PROJECT_NAME}/InesctecMrdtHangfaDiscoveryQ2.cpp
    src/${PROJECT_NAME}/InesctecMrdtHangfaDiscoveryQ2DriverROS1.cpp
    src/${PROJECT_NAME}/SerialChannelsConfig.cpp
    src/main_ros1.cpp
  )

  # - target dependencies
  add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  # - specify linking libraries
  target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  # - installation rules
  install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )

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
