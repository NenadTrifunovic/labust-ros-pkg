cmake_minimum_required(VERSION 2.8.3)
project(labust_docking)

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

add_executable(docking src/docking.cpp)
target_link_libraries(docking ${catkin_LIBRARIES})

## Install
install(TARGETS docking
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
