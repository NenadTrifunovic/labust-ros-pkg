cmake_minimum_required(VERSION 2.8.3)
project(labust_navigation)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  auv_msgs
  roscpp
  snippets
  sensor_msgs
  labust_msgs
  geometry_msgs
  tf2_ros
  underwater_msgs
  labust_diagnostics
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)

## Catkin specific configuration 
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES labust_navigation
  CATKIN_DEPENDS auv_msgs roscpp snippets underwater_msgs geometry_msgs labust_msgs sensor_msgs tf2_ros
)

### Build 
#include_directories(include)
#include_directories(
#  ${catkin_INCLUDE_DIRS}
#  ${Eigen_INCLUDE_DIRS}
#)
#
#add_library(labust_navigation
#   src/labust_navigation/KinematicModel.cpp
#   src/labust_navigation/XYModel.cpp
#   src/labust_navigation/SensorHandlers.cpp
#   src/labust_navigation/LDTravModelExtended.cpp
#   src/labust_navigation/EKF_3D_USBLModel.cpp 
#)
#target_link_libraries(labust_navigation
#   ${catkin_LIBRARIES}
#   ${GeographicLib_LIBRARIES}
#)
#
## Navframe nodes
#add_executable(llnode src/latlon_node/LLNode.cpp)
#target_link_libraries(llnode ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})
#
#add_executable(latlon2ned src/latlon_node/latlon2ned.cpp)
#target_link_libraries(latlon2ned ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})
#
#add_executable(ltp_node src/latlon_node/ltp_node.cpp)
#target_link_libraries(ltp_node ${catkin_LIBRARIES})
#
#add_executable(tf_publisher_node src/nav_node/tf_publisher_node.cpp)
#target_link_libraries(tf_publisher_node ${catkin_LIBRARIES})
#
## Filters 
#add_executable(ekf3d_node src/nav_node/EKF3D.cpp)
#target_link_libraries(ekf3d_node ${catkin_LIBRARIES} labust_navigation)
#
#add_executable(ekf_3d_usbl_node src/nav_node/EKF_3D_USBL.cpp)
#target_link_libraries(ekf_3d_usbl_node ${catkin_LIBRARIES} labust_navigation)
#
### Install
#install(TARGETS labust_navigation llnode latlon2ned ltp_node tf_publisher_node ekf3d_node ekf_3d_usbl_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
#install(DIRECTORY include/labust
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
#)

