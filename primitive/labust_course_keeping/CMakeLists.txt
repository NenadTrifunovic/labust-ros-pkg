cmake_minimum_required(VERSION 2.8.3)
project(labust_course_keeping)

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
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(course_keeping src/course_keeping.cpp)
target_link_libraries(course_keeping ${catkin_LIBRARIES})

## Install
install(TARGETS course_keeping
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/labust
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

