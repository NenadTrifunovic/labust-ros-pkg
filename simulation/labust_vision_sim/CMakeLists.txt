cmake_minimum_required(VERSION 2.8.3)
project(labust_vision_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  labust_msgs
  labust_sim_core
  tf2_ros
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS random)

## Catkin specific configuration
catkin_package()

## Build
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}   
  ${catkin_INCLUDE_DIRS}
)

add_executable(sonar_sim_node src/sonar_sim_node.cpp)
target_link_libraries(sonar_sim_node ${catkin_LIBRARIES} ${Boost_RANDOM_LIBRARY})

## Install
install(TARGETS sonar_sim_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
