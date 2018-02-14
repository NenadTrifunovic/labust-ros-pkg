cmake_minimum_required(VERSION 2.8.3)
project(labust_es_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cmake_modules  
  auv_msgs
  labust_msgs
  sensor_msgs
  dynamic_reconfigure
  snippets
  tf2_ros
  labust_control  
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    cfg/TDOASSControl.cfg
)

## Catkin specific configuration
catkin_package()

## Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(esctoolkit
   src/labust_esctoolkit/esc_classic.cpp
)

# TDOASS node
add_executable(tdoass_control src/tdoass/tdoass_control.cpp)
add_dependencies(tdoass_control labust_es_control_gencfg)
target_link_libraries(tdoass_control ${catkin_LIBRARIES} esctoolkit)

## Install
install(TARGETS  esctoolkit tdoass_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/labust
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
