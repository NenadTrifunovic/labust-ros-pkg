cmake_minimum_required(VERSION 2.8.3)
project(labust_primitive)

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
)

## Catkin specific configuration
catkin_package(INCLUDE_DIRS include)

## Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Install

install(DIRECTORY include/labust
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
