cmake_minimum_required(VERSION 3.8)
project(rs_monitor)

include(FetchContent)

set(MONITOR_LIBRARY_NAME "${PROJECT_NAME}_library")
set(MONITOR_EXECUTABLE_NAME "${PROJECT_NAME}")

# set cppstandard to 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS True)
  # fetch yaml-cpp from github
  FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG 0.8.0
  )
  FetchContent_MakeAvailable(yaml-cpp)
else()
  find_package(yaml-cpp REQUIRED)
endif()

add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

# Detect ros version
if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  find_package(catkin QUIET)
  find_package(ament_cmake QUIET)

  if(catkin_FOUND)
    set(ROS_VERSION 1)
  elseif(ament_cmake_FOUND)
    set(ROS_VERSION 2)
  else()
    message(FATAL_ERROR "Can not detect ROS version automatically")
  endif()
endif()

message(STATUS "Building for ROS${ROS_VERSION}")

# Build for ROS1
if(${ROS_VERSION} EQUAL 1)
  add_compile_definitions(__ROS1__=1)
  add_compile_definitions(__ROS2__=0)

  find_package(catkin REQUIRED
    rospy
    roscpp
    diagnostic_msgs
    topic_tools
    roslib
  )

  catkin_package(
    INCLUDE_DIRS
      include
    LIBRARIES
      ${MONITOR_LIBRARY_NAME}
    CATKIN_DEPENDS
      rospy roscpp diagnostic_msgs topic_tools roslib
  )

  include_directories(${catkin_INCLUDE_DIRS})

# Build for ROS2
elseif(${ROS_VERSION} EQUAL 2)
  add_compile_definitions(__ROS1__=0)
  add_compile_definitions(__ROS2__=1)

  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(rclpy REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(diagnostic_msgs REQUIRED)

  # add macros of ros2 version
  message(STATUS "Using rclcpp version: [${rclcpp_VERSION}]")

  # parse version and add macros
  string(REPLACE "." ";" RCLCPP_VERSION_LIST ${rclcpp_VERSION})
  list(GET RCLCPP_VERSION_LIST 0 RCLCPP_VERSION_MAJOR)
  list(GET RCLCPP_VERSION_LIST 1 RCLCPP_VERSION_MINOR)
  list(GET RCLCPP_VERSION_LIST 2 RCLCPP_VERSION_PATCH)

  add_compile_definitions(RCLCPP_VERSION_MAJOR=${RCLCPP_VERSION_MAJOR})
  add_compile_definitions(RCLCPP_VERSION_MINOR=${RCLCPP_VERSION_MINOR})
  add_compile_definitions(RCLCPP_VERSION_PATCH=${RCLCPP_VERSION_PATCH})

  # add distribution macros
  if(${RCLCPP_VERSION_MAJOR} LESS 9)
    add_definitions(-DROS2_FOXY)
  elseif(${RCLCPP_VERSION_MAJOR} LESS 16)
    add_definitions(-DROS2_GALACTIC)
  elseif(${RCLCPP_VERSION_MAJOR} LESS 20)
    add_definitions(-DROS2_HUMBLE)
  elseif(${RCLCPP_VERSION_MAJOR} LESS 24)
    add_definitions(-DROS2_IRON)
  elseif(${RCLCPP_VERSION_MAJOR} LESS 29)
    add_definitions(-DROS2_JAZZY)
  elseif()
    add_definitions(-DROS2_ROLLING)
  endif()

endif()

# file(GLOB_RECURSE RS_MONITOR_SROUCE ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
# # compile node executable only for ROS1
# list(REMOVE_ITEM RS_MONITOR_SROUCE ${CMAKE_CURRENT_SOURCE_DIR}/src/node.cpp)

set(RS_MONITOR_SROUCE
  src/common/common.cpp
  src/common/ros2_process_manager.cpp
  src/common/system_stats_linux.cpp
  src/common/system_stats_windows.cpp
  src/monitor/monitor_base.cpp
  src/monitor/frequency_monitor.cpp
  src/monitor/resource_monitor.cpp
  src/monitor/timestamp_monitor.cpp
  src/monitor_manager.cpp
)

add_library(${MONITOR_LIBRARY_NAME} SHARED ${RS_MONITOR_SROUCE})
target_include_directories(${MONITOR_LIBRARY_NAME} PUBLIC include)

if(WIN32)
  target_link_libraries(${MONITOR_LIBRARY_NAME} pdh.lib psapi.lib)
  target_link_libraries(${MONITOR_LIBRARY_NAME} yaml-cpp::yaml-cpp)
else()
  target_link_libraries(${MONITOR_LIBRARY_NAME} ${YAML_CPP_LIBRARIES})
endif()

if(ROS_VERSION EQUAL 1)
  target_link_libraries(${MONITOR_LIBRARY_NAME} ${catkin_LIBRARIES})

  add_executable(${MONITOR_EXECUTABLE_NAME} src/node.cpp)
  target_link_libraries(${MONITOR_EXECUTABLE_NAME} ${MONITOR_LIBRARY_NAME})

  install(
    TARGETS ${MONITOR_LIBRARY_NAME} ${MONITOR_EXECUTABLE_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  catkin_python_setup()

  catkin_install_python(PROGRAMS
    scripts/setup.py scripts/parser.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  install(DIRECTORY rs_monitor DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${PROJECT_NAME})

elseif(ROS_VERSION EQUAL 2)
  ament_target_dependencies(${MONITOR_LIBRARY_NAME}
    rclcpp
    rclcpp_components
    ament_index_cpp
    diagnostic_msgs
  )

  # register component
  rclcpp_components_register_node(
    ${MONITOR_LIBRARY_NAME}
    PLUGIN "robosense::rs_monitor::MonitorManager"
    EXECUTABLE ${MONITOR_EXECUTABLE_NAME}
  )

  # install files
  install(TARGETS
    ${MONITOR_LIBRARY_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

  # install python modules
  ament_python_install_package(${PROJECT_NAME})

  # install python scripts
  install(PROGRAMS
    scripts/setup.py
    scripts/parser.py
    DESTINATION lib/${PROJECT_NAME}
  )

  # test
  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()
endif()
