cmake_minimum_required(VERSION 2.8.3)
project(labust_diver_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  snippets
  labust_msgs
  auv_msgs
  nav_msgs
  sensor_msgs
  labust_sim_core
)

## Catkin specific configuration 
catkin_package()

## Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(diver_sim_node src/diver_sim_node.cpp)
target_link_libraries(diver_sim_node ${catkin_LIBRARIES})

## Install 
install(TARGETS diver_sim_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
