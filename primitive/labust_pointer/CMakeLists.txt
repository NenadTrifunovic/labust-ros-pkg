cmake_minimum_required(VERSION 2.8.3)
project(labust_pointer)

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

catkin_package()

## Build

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(track_diver src/track_diver.cpp)
target_link_libraries(track_diver ${catkin_LIBRARIES})

## Install
## Mark executables and/or libraries for installation
install(TARGETS track_diver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/labust
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
