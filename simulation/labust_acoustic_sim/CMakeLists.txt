cmake_minimum_required(VERSION 2.8.3)
project(labust_acoustic_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2_ros
  snippets
  auv_msgs
  underwater_msgs
)

find_package(Boost REQUIRED COMPONENTS random)

## Catkin specific configuration 
catkin_package()

## Build
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(usbl_sim_node src/usbl_sim_node.cpp)
target_link_libraries(usbl_sim_node ${catkin_LIBRARIES})

add_executable(range_sim_node src/range_sim_node.cpp)
target_link_libraries(range_sim_node ${catkin_LIBRARIES} ${Boost_RANDOM_LIBRARY})

add_executable(tdoa_sim_node src/tdoa_sim_node.cpp)
target_link_libraries(tdoa_sim_node ${catkin_LIBRARIES})

## Install
install(TARGETS usbl_sim_node range_sim_node tdoa_sim_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
