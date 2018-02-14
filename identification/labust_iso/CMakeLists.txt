cmake_minimum_required(VERSION 2.8.3)
project(labust_iso)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  auv_msgs
  actionlib
  snippets
  labust_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES I_SO
   CATKIN_DEPENDS auv_msgs roscpp actionlib snippets
)

## Build
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(I_SO
   src/SOIdentification.cpp
   include/labust/identification/SOIdentification.hpp
   include/labust/identification/Relay.hpp
)

add_executable(ident_so src/IdentificationNode.cpp
    include/labust/identification/IdentificationNode.hpp)
target_link_libraries(ident_so I_SO ${catkin_LIBRARIES})

## Install
install(TARGETS I_SO ident_so
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/labust
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY include/labust
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
)
