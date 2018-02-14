cmake_minimum_required(VERSION 2.8.3)
project(labust_geo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  std_msgs
  snippets  
  tf2_ros
  geometry_msgs
  sensor_msgs
  nav_msgs
  auv_msgs  
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)


## Catkin specific configuration 
catkin_package()

## Build 
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}  
)

add_executable(tf_publish_node src/tf_publish_node.cpp)
target_link_libraries(tf_publish_node
  ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES} ${Eigen_LIBRARIES}
)

add_executable(odom_to_navsts_node src/odom_to_navsts_node.cpp)
target_link_libraries(odom_to_navsts_node
  ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES} ${Eigen_LIBRARIES}
)

## Install 
install(TARGETS odom_to_navsts_node tf_publish_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/labust
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
