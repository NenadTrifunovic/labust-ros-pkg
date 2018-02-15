cmake_minimum_required(VERSION 2.8.3)
project(snippets)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED
    cmake_modules
    roscpp
    rospy
    rosbag
    geometry_msgs
    tf2_geometry_msgs)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed.
catkin_python_setup()

## Catkin specific configuration
catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  CATKIN_DEPENDS geometry_msgs rospy tf2_geometry_msgs
)

## Build
include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS} 
  ${Eigen_INCLUDE_DIRS}
)

## Install
install(
   DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION} 
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(
   DIRECTORY data/locations
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
   FILES_MATCHING PATTERN "*.yaml"
)

## Testing
catkin_add_gtest(snippets-test
 test/test_snippets.cpp
 test/test_bitstorage.cpp)
