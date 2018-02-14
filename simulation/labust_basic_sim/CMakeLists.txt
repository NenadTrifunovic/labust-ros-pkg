cmake_minimum_required(VERSION 2.8.3)
project(labust_basic_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  std_msgs
  snippets
  tf2_ros
  labust_sim_core
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS random)

## Catkin specific configuration 
catkin_package()

## Build 
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}   
)

add_executable(imu_sim_node src/imu_sim_node.cpp)
target_link_libraries(imu_sim_node ${catkin_LIBRARIES} ${Boost_RANDOM_LIBRARY})

add_executable(dvl_sim_node src/dvl_sim_node.cpp)
target_link_libraries(dvl_sim_node ${catkin_LIBRARIES} ${Boost_RANDOM_LIBRARY})

add_executable(gps_sim_node src/gps_sim_node.cpp)
target_link_libraries(gps_sim_node ${catkin_LIBRARIES} Geographic ${Boost_RANDOM_LIBRARY})

## Install
install(TARGETS imu_sim_node dvl_sim_node gps_sim_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
