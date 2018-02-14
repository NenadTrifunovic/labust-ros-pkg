cmake_minimum_required(VERSION 2.8.3)
project(labust_dynamic_positioning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  auv_msgs
  labust_msgs
  tf2_ros
  snippets 
  labust_primitive   
)

## Catkin specific configuration
catkin_package()

## Build 
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dynamic_positioning src/dynamic_positioning.cpp)
target_link_libraries(dynamic_positioning ${catkin_LIBRARIES})

## Install
install(TARGETS dynamic_positioning
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
