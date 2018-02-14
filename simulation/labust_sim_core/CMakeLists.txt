cmake_minimum_required(VERSION 2.8.3)
project(labust_sim_core)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  std_msgs
  snippets 
  auv_msgs
  nav_msgs
  underwater_msgs
  labust_msgs
  tf2_ros   
)

find_package(Boost REQUIRED COMPONENTS random)
find_package(Eigen3 REQUIRED)

## Catkin specific configuration
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES labust_simulation
)

## Build
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Main simulation library
file(GLOB SRC src/labust_simulation/*.cpp)
file(GLOB HPP include/labust/simulation/*.hpp include/labust/simulation/*.h)
add_library(labust_simulation ${SRC} ${HPP})
target_link_libraries(labust_simulation ${Boost_RANDOM_LIBRARY} ${Eigen_LIBRARIES})

# Main simulator node
file(GLOB SRC src/*.cpp)
file(GLOB HPP include/labust/simulation/*.hpp)
add_executable(sim_core_node ${SRC} ${HPP})
target_link_libraries(sim_core_node ${catkin_LIBRARIES} labust_simulation ${Boost_RANDOM_LIBRARY})

## Install
install(TARGETS sim_core_node labust_simulation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/labust
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
