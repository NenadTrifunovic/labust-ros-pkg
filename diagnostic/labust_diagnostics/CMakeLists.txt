cmake_minimum_required(VERSION 2.8.3)
project(labust_diagnostics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  diagnostic_msgs
)

catkin_python_setup()

## Catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  labust_diagnostics
  CATKIN_DEPENDS std_msgs sensor_msgs diagnostic_msgs rospy roscpp
)

## Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB SRC src/StatusHandler.cpp)
file(GLOB HPP)
add_library(labust_diagnostics ${SRC} ${HPP})
target_link_libraries(labust_diagnostics ${catkin_LIBRARIES})

add_executable(diagnostic_node src/diagnostic_node.cpp)
target_link_libraries(diagnostic_node labust_diagnostics ${catkin_LIBRARIES})

## Install
install(TARGETS labust_diagnostics diagnostic_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  FILES_MATCHING PATTERN "*"
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
              GROUP_EXECUTE GROUP_READ
              WORLD_EXECUTE WORLD_READ
)

install(DIRECTORY include/labust
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
